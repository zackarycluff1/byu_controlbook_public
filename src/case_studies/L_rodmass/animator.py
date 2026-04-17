# 3rd-party
import matplotlib.patches as mpatches
import numpy as np

# local (controlbook)
from ..common.animator import MatplotlibAxisAnimator


class RodMassAnimator(MatplotlibAxisAnimator):
    """
    Animator for the rod-mass system.
    
    Visualizes a point mass attached to a massless rod, connected to a wall
    with a nonlinear spring and damper. The rod rotates about the wall attachment.
    
    By inheriting from MatplotlibAxisAnimator, this class has access to the
    matplotlib axis object via `self.ax`. This class implements the three
    required methods: setup_background(), setup_changing_objects(), and
    update_changing_objects().
    """

    def setup_background(self):
        """
        Called once at initialization to set up static visual elements.
        """
        # Drawing parameters
        self.L = 1.0      # rod length for visualization
        self.w = 0.01     # rod width
        self.R = 0.1      # bob radius
        self.H = 1.0      # wall height
        
        # Set axis limits and aspect ratio
        lim = 2.0 * self.L
        self.ax.axis((-lim, lim, -lim, lim))
        self.ax.set_aspect("equal", "box")
        
        # Draw wall (static background element)
        corner = (-self.H / 3.0, -self.H)  # bottom left corner
        wall = mpatches.Rectangle(
            corner, width=self.H / 3.0, height=2.0 * self.H,
            fc='blue', ec='black'
        )
        self.ax.add_patch(wall)
        
        # Draw reference line (horizontal baseline)
        self.ax.plot([0, self.L], [0, 0], 'k--', linewidth=1)

    def setup_changing_objects(self, x0, r0):
        """
        Called once at initialization to create animated objects.
        
        Args:
            x0: Initial state vector [θ₀, θ̇₀]
            r0: Initial reference (not used in this animation)
        """
        theta0 = x0[0]
        
        # Create rod (line object)
        rod_x, rod_y = self.get_rod_endpoints(theta0)
        (self.rod_line,) = self.ax.plot(rod_x, rod_y, linewidth=3, color="black")
        
        # Create bob (circle patch) - use Circle instead of CirclePolygon for easier updates
        bob_center = self.get_bob_center(theta0)
        self.bob_patch = mpatches.Circle(
            bob_center, radius=self.R,
            fc='limegreen', ec='black'
        )
        self.ax.add_patch(self.bob_patch)

    def update_changing_objects(self, x, r):
        """
        Called every frame to update animated objects with new state.
        
        Args:
            x: Current state vector [θ, θ̇]
            r: Current reference (not used in this animation)
            
        Returns:
            Tuple of artists that changed (for blitting)
        """
        theta = x[0]
        
        # Update rod position
        rod_x, rod_y = self.get_rod_endpoints(theta)
        self.rod_line.set_xdata(rod_x)
        self.rod_line.set_ydata(rod_y)
        
        # Update bob position - Circle patches have a set_center() method
        bob_center = self.get_bob_center(theta)
        self.bob_patch.set_center(bob_center)
        
        return (self.rod_line, self.bob_patch)

    # Helper methods (system-specific geometry calculations)
    
    def get_rod_endpoints(self, theta):
        """
        Calculate rod endpoints for given angle.
        
        The rod is drawn as a rectangle rotated by theta. This creates
        the 4 corner points and returns the x, y arrays.
        
        Args:
            theta: Angle from horizontal (rad)
            
        Returns:
            tuple: (x_coords, y_coords) arrays of rod corners
        """
        # Define rod as rectangle in local coordinates
        pts = np.array([
            [0, self.L, self.L, 0],
            [-self.w / 2, -self.w / 2, self.w / 2, self.w / 2]
        ])
        
        # Rotation matrix
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        
        # Apply rotation
        pts_rotated = R @ pts
        return pts_rotated[0], pts_rotated[1]

    def get_bob_center(self, theta):
        """
        Calculate bob center position for given angle.
        
        Args:
            theta: Angle from horizontal (rad)
            
        Returns:
            tuple: (x, y) coordinates of bob center
        """
        x = (self.L + self.R) * np.cos(theta)
        y = (self.L + self.R) * np.sin(theta)
        return (x, y)
