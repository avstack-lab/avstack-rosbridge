from avstack.geometry import Position, Velocity, Attitude, Vector, Rotation


class GeometryBridge:
    @staticmethod
    def _vector_to_avstack(aclass: Vector, msg, reference):
        pass

    @staticmethod
    def _rotation_to_avstack(aclass: Rotation):
        pass

    def position_to_avstack(self):
        return self.

    def attitude_to_avstack(self):
        pass

    def velocity_to_avstack(self):
        pass

    def acceleration_to_avstack(self):
        pass

    def angular_vel_to_avstack(self):
        pass