################################################################################
# Originally created by R.W. Beard (12/2022)
################################################################################

# 3rd-party
import numpy as np
from pyqtgraph import opengl as gl
from pyqtgraph.Vector import Vector
import time

# local (controlbook)
from case_studies.common.animator import OpenglWidgetAnimator


mygrey1 = np.array([0.8, 0.8, 0.8, 1], dtype=np.float32)  # light
mygrey2 = np.array([0.6, 0.6, 0.6, 1], dtype=np.float32)
mygrey3 = np.array([0.5, 0.5, 0.5, 1], dtype=np.float32)
mygrey4 = np.array([0.3, 0.3, 0.3, 1], dtype=np.float32)  # dark


class HummingbirdAnimator(OpenglWidgetAnimator):
    """
    By inheriting from OpenglWidgetAnimator, this class has access to the
    opengl widget object via `self.widget`. This class just needs to implement
    the `setup_background()`, `setup_changing_objects()`, and
    `update_changing_objects()` methods.
    """

    def setup_background(self) -> None:
        grid = gl.GLGridItem()  # make a grid to represent the ground
        grid.scale(x=20, y=20, z=20)
        self.widget.addItem(grid)
        self.widget.setCameraPosition(distance=15)
        self.widget.setBackgroundColor("k")  # black
        self.widget.opts["center"] = Vector(0, 0, 0)  # center camera on origin

    def setup_changing_objects(self, x0, r0) -> None:
        unit_length = 4.0
        base_width = 0.1 * unit_length
        base_height = 0.1 * unit_length
        base_length = 0.6 * unit_length
        cw_width = 0.2 * unit_length  # cw==counterweight
        cw_height = 0.2 * unit_length
        cw_length = 0.4 * unit_length
        body_width = 0.1 * unit_length
        body_height = 0.1 * unit_length
        body_length = 0.4 * unit_length
        arm_width = 0.05 * unit_length
        arm_height = 0.05 * unit_length
        arm_length = 0.8 * unit_length
        motor_width = 0.6 * arm_width
        motor_height = 0.6 * arm_height
        motor_length = 2 * motor_width

        self.cw_pos_local = np.array([0, 0, -base_length])
        self.body_pos_local = np.array([(cw_length + body_length) / 2, 0, 0])
        self.arm_pos_local = np.array([body_length / 2, 0, 0])
        self.motor1_pos_local = np.array(
            [
                (arm_length - motor_width) / 2,
                0,
                -(arm_height + motor_length) / 2,
            ]
        )
        self.motor2_pos_local = self.motor1_pos_local * np.array([-1, 1, 1])

        # base
        self.base = RectangularPrismViz(
            base_width,
            base_height,
            base_length,
            center=np.array([0, 0, -base_length / 2]),
            R=euler2R(0, np.pi / 2, 0),
        )
        self.widget.addItem(self.base.viz)

        # counterweight
        self.counterweight = RectangularPrismViz(cw_width, cw_height, cw_length)
        self.widget.addItem(self.counterweight.viz)

        # body
        self.body = RectangularPrismViz(body_width, body_height, body_length)
        self.widget.addItem(self.body.viz)

        # arm
        self.arm = RectangularPrismViz(arm_width, arm_height, arm_length)
        self.widget.addItem(self.arm.viz)

        # motors
        self.motor1 = RectangularPrismViz(motor_width, motor_height, motor_length)
        self.widget.addItem(self.motor1.viz)
        self.motor2 = RectangularPrismViz(motor_width, motor_height, motor_length)
        self.widget.addItem(self.motor2.viz)

        # rotors
        radius = 5 * arm_width
        self.rotor1 = RotorViz(radius)
        self.widget.addItem(self.rotor1.viz)
        self.rotor2 = RotorViz(radius)
        self.widget.addItem(self.rotor2.viz)

        self.update_changing_objects(x0, r0)

    def update_changing_objects(self, x, r) -> None:
        phi, theta, psi = x[:3]

        # counterweight
        cw_rotation = euler2R(0, theta, psi)
        self.counterweight.update_data(self.cw_pos_local, cw_rotation)

        # body
        body_position = self.cw_pos_local + cw_rotation @ self.body_pos_local
        body_rotation = euler2R(phi, theta, psi)
        self.body.update_data(body_position, body_rotation)

        # arm
        arm_position = body_position + body_rotation @ self.arm_pos_local
        arm_rotation = euler2R(phi, theta, psi) @ euler2R(0, 0, np.pi / 2)
        self.arm.update_data(arm_position, arm_rotation)

        # motors
        motor1_position = arm_position + arm_rotation @ self.motor1_pos_local
        motor1_rotation = euler2R(phi, theta, psi) @ euler2R(0, np.pi / 2, 0)
        self.motor1.update_data(motor1_position, motor1_rotation)

        motor2_position = arm_position + arm_rotation @ self.motor2_pos_local
        motor2_rotation = euler2R(phi, theta, psi) @ euler2R(0, np.pi / 2, 0)
        self.motor2.update_data(motor2_position, motor2_rotation)

        # rotors
        rotor_spin = 10 * time.time()
        rotor1_rotation = euler2R(phi, theta, psi) @ euler2R(0, 0, rotor_spin)
        self.rotor1.update_data(motor1_position, rotor1_rotation)

        rotor2_rotation = euler2R(phi, theta, psi) @ euler2R(0, 0, -rotor_spin)
        self.rotor2.update_data(motor2_position, rotor2_rotation)


def euler2R(phi, theta, psi):
    """
    Converts euler angles to rotation matrix: R_b^i = Rz(psi)Ry(theta)Rx(phi).
    """
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)
    R_roll = np.array([[1, 0, 0], [0, c_phi, -s_phi], [0, s_phi, c_phi]])
    R_pitch = np.array([[c_theta, 0, s_theta], [0, 1, 0], [-s_theta, 0, c_theta]])
    R_yaw = np.array([[c_psi, -s_psi, 0], [s_psi, c_psi, 0], [0, 0, 1]])
    R = R_yaw @ R_pitch @ R_roll
    return R


def ned2enu(points_ned):
    """
    Converts points from NED to ENU frame.
    """
    R_ned2enu = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points_enu = R_ned2enu @ points_ned
    return points_enu


class MeshViz:
    def __init__(self, vertices, faces, colors, center=np.zeros(3), R=np.eye(3)):
        self.vertices = vertices
        self.faces = faces
        self.colors = colors
        mesh = self._points_to_mesh(vertices, faces, center, R)
        self.viz = gl.GLMeshItem(
            vertexes=mesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=colors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False,
        )

    def update_data(self, center, R):
        mesh = self._points_to_mesh(self.vertices, self.faces, center, R)
        self.viz.setMeshData(vertexes=mesh, vertexColors=self.colors)

    def _points_to_mesh(self, vertices_body, faces, center, R):
        """
        Transform points from local body from to global ENU frame.

        Args:
            vertices_body (3xN array): points defining the object in its body frame
            faces (Nx3 array): indices defining triangular mesh faces, e.g., [0,3,4]
                makes a triangle face from vertices 0, 3, and 4
            center (3 array): position vector from global origin to center of body frame
            R (3x3 array): rotation matrix from object's body frame to global NED frame
        Returns:
            mesh (Nx3x3 array): mesh points in global ENU frame
        """
        points_local_ned = R @ vertices_body
        points_global_ned = points_local_ned + center.reshape(-1, 1)
        points_global_enu = ned2enu(points_global_ned)
        mesh = points_global_enu.T[faces]
        return mesh


class RotorViz(MeshViz):
    num_segments = 16
    num_blades = 4
    angles = np.linspace(0, 2 * np.pi, num_segments + 1, endpoint=True)
    vertices = np.block(
        [
            [0, np.cos(angles)],
            [0, np.sin(angles)],
            [np.zeros(num_segments + 2)],
        ]
    )
    # faces connects center vertex to pair of adjacent outer vertices
    faces = np.array([[0, i, i + 1] for i in range(0, num_segments + 1, num_blades)])
    mesh_colors = np.tile(mygrey4, (len(faces), 3, 1))

    ## TODO: decide which version to use
    ## above draws individual blades only
    ## below draws white blades between dark sections
    # faces = np.array([[0, i, i + 1] for i in range(num_segments + 1)])
    # mesh_colors = np.tile(mygrey4, (len(faces), 3, 1))
    # step = num_segments // num_blades
    # mesh_colors[::step, :, :3] = 0.7

    def __init__(self, radius):
        vertices = self.vertices * radius
        super().__init__(vertices, self.faces, self.mesh_colors)


class RectangularPrismViz(MeshViz):
    # corners are in ENU coordinates assuming center at origin
    corners = 0.5 * np.array(
        [
            [1, -1, -1],
            [1, 1, -1],
            [1, 1, 1],
            [1, -1, 1],
            [-1, -1, -1],
            [-1, 1, -1],
            [-1, 1, 1],
            [-1, -1, 1],
        ]
    )
    faces = np.array(
        [
            [6, 3, 2],  # top
            [6, 3, 7],  # top
            [7, 0, 3],  # right side
            [7, 0, 4],  # right side
            [6, 1, 2],  # left side
            [6, 1, 5],  # left side
            [5, 0, 1],  # bottom
            [5, 0, 4],  # bottom
            [3, 1, 2],  # front
            [3, 1, 0],  # front
            [7, 5, 6],  # back
            [7, 5, 4],  # back
        ]
    )
    mesh_colors = np.empty((12, 3, 4), dtype=np.float32)
    mesh_colors[0] = mygrey1  # top
    mesh_colors[1] = mygrey1  # top
    mesh_colors[2] = mygrey1  # right side
    mesh_colors[3] = mygrey1  # right side
    mesh_colors[4] = mygrey1  # left side
    mesh_colors[5] = mygrey1  # left side
    mesh_colors[6] = mygrey4  # bottom
    mesh_colors[7] = mygrey4  # bottom
    mesh_colors[8] = mygrey2  # front
    mesh_colors[9] = mygrey2  # front
    mesh_colors[10] = mygrey3  # back
    mesh_colors[11] = mygrey3  # back

    def __init__(self, width, height, length, center=np.zeros(3), R=np.eye(3)):
        vertexes = (self.corners * [length, width, height]).T
        super().__init__(vertexes, self.faces, self.mesh_colors, center, R)
