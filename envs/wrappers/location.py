class Location:
    def __init__(self, loc_string):
        # Format: "LOCATION_NAME LOCATION_TYPE CLEAR?"
        loc_info = loc_string.split(" ")
        self.loc_name = loc_info[1]
        self.loc_type = loc_info[0]  # dirty-area, clean-area, coffee-machine, or serving-counter

    def __str__(self):
        return f"{self.loc_name}"

