class Button:
    def __init__(self, but_string):
        # Format: "BUTTON_TYPE BUTTON_NAME"
        button_info = but_string.split(" ")
        self.button_type = button_info[0]
        self.button_name = button_info[1]

    def __str__(self):
        return f"{self.button_name}"