
class Node:
    def __init__(self) -> None:
        self.prev = None
        self.val = 0.0
        self.x = None
        self.y = None
        self.vel = 0.0
        self.steps = 0.0
        self.vehicle = None
        self.state_idx = None

    def inherit(self, node) -> None:
        for k, v in node.__dict__.items():
            if k == "vehicle":
                v = v.copy() # avoid reference errors
            self.__dict__[k] = v

        self.prev = node
        return self

    def get_location(self):
        return self.x, self.y

    def update(self, state, score) -> None:
        self.steps += 1
        self.x, self.y = state
        self.val = score
        self.vehicle.update(self)
        self.vel = self.vehicle.v

    def __str__(self) -> str:
        return f"Node value {self.val} with step cnt {self.steps} \n {self.vehicle}"