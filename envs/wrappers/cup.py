class Cup:
    def __init__(self, cup_string):
        # Format: "CUP_TYPE CUP_NAME CUP_POS CLEAN? EMPTY? FULL?"
        cup_info = cup_string.split(" ")
        self.cup_type = cup_info[0]
        self.cup_name = cup_info[1]
        self.cup_loc = cup_info[2]
        self.cup_clean = cup_info[3] == "True"
        self.cup_empty = cup_info[4] == "True"
        self.cup_full = cup_info[5] == "True"
        self.dirty = cup_info[6] == "True"

    def __str__(self):
        return f"{self.cup_type} {self.cup_name} {self.cup_loc}"