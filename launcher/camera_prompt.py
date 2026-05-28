import copy
import customtkinter as ctk


def prompt_camera_assignment_window(parent, camera_candidates):
    """Show camera role assignment popup and return pending role assignments.

    Returns:
        list[dict]: Each item has:
            - role: "disabled", "wrist", "workspace", "side", or "overhead"
            - busid: Windows usbipd BUSID or mock BUSID
            - usb_name: friendly camera name
    """
    if not camera_candidates:
        return []

    role_values = ["disabled", "wrist", "workspace", "side", "overhead"]
    default_roles = ["wrist", "workspace", "side", "overhead"]
    candidates = copy.deepcopy(camera_candidates)

    win = ctk.CTkToplevel(parent)
    win.title("Assign Camera Roles")
    win.geometry("720x560")
    win.transient(parent)
    win.grab_set()

    ctk.CTkLabel(
        win,
        text="Assign Camera Roles",
        font=("Arial", 18, "bold"),
    ).pack(anchor="w", padx=20, pady=(20, 5))

    ctk.CTkLabel(
        win,
        text=(
            "Choose camera roles before USB devices are attached to WSL. "
            "You can leave this window open as long as needed. "
            "If many cameras are listed, resize/expand this window to see all rows and the Done button. "
            "Disabled cameras will not be attached or written to config."
        ),
        wraplength=660,
    ).pack(anchor="w", padx=20, pady=(0, 15))

    warning_label = ctk.CTkLabel(win, text="", text_color="#ffcc66")
    warning_label.pack(anchor="w", padx=20, pady=(0, 5))

    rows_frame = ctk.CTkFrame(win)
    rows_frame.pack(fill="both", expand=True, padx=20, pady=10)

    role_vars = []
    previous_roles = []
    updating_roles = {"active": False}

    def first_available_role(exclude_roles=None):
        exclude_roles = set(exclude_roles or [])
        used = {var.get() for var in role_vars if var.get() != "disabled"}

        for role in role_values:
            if role == "disabled":
                continue
            if role not in used and role not in exclude_roles:
                return role

        return "disabled"

    def snapshot_roles():
        for idx, var in enumerate(role_vars):
            previous_roles[idx] = var.get()

    def on_role_selected(camera_index, selected_role):
        if updating_roles["active"]:
            return

        updating_roles["active"] = True
        try:
            old_role = previous_roles[camera_index]

            if selected_role != "disabled":
                for other_index, other_var in enumerate(role_vars):
                    if other_index == camera_index:
                        continue

                    if other_var.get() == selected_role:
                        replacement_role = old_role

                        if replacement_role == "disabled" or replacement_role == selected_role:
                            replacement_role = first_available_role(exclude_roles={selected_role})

                        other_var.set(replacement_role)
                        break

            snapshot_roles()
            warning_label.configure(text="")
        finally:
            updating_roles["active"] = False

    for i, cam in enumerate(candidates):
        row = ctk.CTkFrame(rows_frame, fg_color="transparent")
        row.pack(fill="x", padx=10, pady=6)

        label_text = "Camera {}: {}".format(
            i + 1,
            cam.get("usb_name") or cam.get("busid") or f"index {i}",
        )

        ctk.CTkLabel(
            row,
            text=label_text,
            width=390,
            anchor="w",
        ).pack(side="left", padx=(0, 10))

        default_role = default_roles[i] if i < len(default_roles) else "disabled"
        var = ctk.StringVar(value=default_role)
        role_vars.append(var)
        previous_roles.append(default_role)

        menu = ctk.CTkOptionMenu(
            row,
            variable=var,
            values=role_values,
            width=170,
            command=lambda choice, idx=i: on_role_selected(idx, choice),
        )
        menu.pack(side="left")

    def validate_roles():
        selected = [var.get() for var in role_vars]
        used = set()
        duplicates = []

        for role in selected:
            if role == "disabled":
                continue

            if role in used:
                duplicates.append(role)

            used.add(role)

        if duplicates:
            warning_label.configure(
                text="Duplicate role still exists: " + ", ".join(sorted(set(duplicates)))
            )
            return False

        if not any(role != "disabled" for role in selected):
            warning_label.configure(
                text="At least one camera must be enabled. Extra cameras can stay disabled."
            )
            return False

        warning_label.configure(text="")
        return True

    def on_done():
        if not validate_roles():
            return

        win.destroy()

    button_row = ctk.CTkFrame(win, fg_color="transparent")
    button_row.pack(fill="x", padx=20, pady=(5, 20))

    ctk.CTkButton(
        button_row,
        text="Done",
        fg_color="green",
        hover_color="darkgreen",
        command=on_done,
    ).pack(side="right", padx=5)

    parent.wait_window(win)

    pending_roles = []
    for i, cam in enumerate(candidates):
        role = role_vars[i].get()
        pending_roles.append(
            {
                "role": role,
                "busid": cam.get("busid"),
                "usb_name": cam.get("usb_name", ""),
            }
        )

    return pending_roles
