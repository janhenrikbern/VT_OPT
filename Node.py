
class Node:
    def __init__(self) -> None:
        self.prev = None
        self.val = 0
        self.x = None
        self.y = None
        self.vel = 0
        self.steps = 0
        self.vehicle = None

    def inherit(self, node) -> None:
        for k, v in node.__dict__.items():
            self.__dict__[k] = v

        self.prev = node
        return self

    def update(self, state, score):
        self.steps += 1
        self.x, self.y = state
        self.val = score
        self.vehicle.update(self)

    def __str__(self) -> str:
        return f"Node value {self.val} with step cnt {self.steps} \n {self.vehicle}"