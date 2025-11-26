import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from scipy.spatial.transform import Rotation as R

from viewer.menu import GUI
from viewer.robot_trace import RobotTrace
from viewer.obj_manager import load_obj_with_tex, create_vertex_data, create_vbo, draw_vbo_textured

import time

RAD2DEG = 180 / 3.1415926535
DEG2RAD = 180 / 3.1415926535
class Viewer:
	def __init__(self, robot, timestep, window_width=1600, window_height=900):
		self.window_width = window_width
		self.window_height = window_height

		self.robot = robot
		self.dt = timestep

		# Playback & animation
		self.etas = None
		self.nus = None
		self.desired_tfs = None
		self.times = None
		self.frame_index = 0
		self.frame_index_float = 0
		self.playback_speed = 1
		self.running = True
		self.step_request = False
		self.fps = 0
		self.initialized = False
		self.current_time = time.time()
		self.last_time = time.time()
		self.fps_last_time = time.time()
		self.frame_count = 0

		# OpenGL resources
		self.robot_vbo = None
		self.robot_texture_id = None
		self.robot_vertex_count = None

		# Camera control
		self.camera_radius = 10
		self.camera_theta = np.pi / 4
		self.camera_phi = np.pi / 4
		self.pan_x, self.pan_y, self.pan_z = 0.0, 0.0, 0.0
		self.mouse_prev = [0, 0]
		self.mouse_button = None
		self.follow_robot = True

		# Other UI/State
		self.gui = None
		self.robot_trace = None

	def load_sequences(self):
		self.etas = self.robot.logger.etas
		self.nus = self.robot.logger.nus
		self.desired_tfs = self.robot.logger.desired_tfs
		self.times = self.robot.logger.timestamps

	def draw_target_marker(self, size=0.05):
		"""Draws a stylized 3D target marker with sphere and axis arrows."""
		# Draw central sphere
		glDisable(GL_LIGHTING)
		glColor3f(1.0, 0.9, 0.0)  # yellowish
		glutSolidSphere(size, 20, 20)
		glEnable(GL_LIGHTING)

	def draw_axes(self, length=0.5, line_width=1.0, draw_on_top=False):
		glDisable(GL_LIGHTING)
		if draw_on_top:
			glDisable(GL_DEPTH_TEST)  # Disable depth test to draw on top
		
		glLineWidth(line_width)  # Set thicker line width
		glBegin(GL_LINES)
		# X axis - red
		glColor3f(1, 0, 0)
		glVertex3f(0, 0, 0)
		glVertex3f(length, 0, 0)
		# Y axis - green
		glColor3f(0, 1, 0)
		glVertex3f(0, 0, 0)
		glVertex3f(0, length, 0)
		# Z axis - blue
		glColor3f(0, 0, 1)
		glVertex3f(0, 0, 0)
		glVertex3f(0, 0, length)
		glEnd()
		glLineWidth(1.0)  # Reset line width
		
		if draw_on_top:
			glEnable(GL_DEPTH_TEST)  # Re-enable depth test
		
		glEnable(GL_LIGHTING)

	def draw_thrusters_thrust(self, length=1.0, line_width=1.0, draw_on_top=False):
		glDisable(GL_LIGHTING)
		if draw_on_top:
			glDisable(GL_DEPTH_TEST)  # Disable depth test to draw on top
		thrusters = self.robot.thrusters
		robot_pos = self.etas[self.frame_index, :3]
		rotation_matrix = R.from_euler('xyz', self.etas[self.frame_index, 3:]).as_matrix()


		glLineWidth(line_width)  # Set thicker line width
		glBegin(GL_LINES)
		
		for i in range(thrusters.n_thrusters):
			thruster_pos = robot_pos + rotation_matrix @ thrusters.positions[i, :3]
			max_abs_thrust = np.max(np.abs(thrusters.limits[i]))
			thrust = self.robot.logger.thruster_forces[self.frame_index, i]
			thrust_ratio = thrust / max_abs_thrust
			max_thrust_pos = robot_pos + rotation_matrix @ (thrusters.positions[i, :3] + length * thrust_ratio * thrusters.T[:3, i])

			# Thrusters
			glColor3f(1, 1, 0)
			glVertex3f(*thruster_pos)
			glVertex3f(*max_thrust_pos)
		
		glEnd()
		glLineWidth(1.0)  # Reset line width
		
		if draw_on_top:
			glEnable(GL_DEPTH_TEST)  # Re-enable depth test
		
		glEnable(GL_LIGHTING)


	def draw_robot_force(self, length=0.5, line_width=1.0, draw_on_top=False):
		glDisable(GL_LIGHTING)
		if draw_on_top:
			glDisable(GL_DEPTH_TEST)  # Disable depth test to draw on top
		
		robot_pos = self.etas[self.frame_index, :3]
		rotation_matrix = R.from_euler('xyz', self.etas[self.frame_index, 3:]).as_matrix()
		thrust_forces = rotation_matrix @ self.robot.logger.thrust_forces[self.frame_index, :3]
		total_forces = rotation_matrix @ self.robot.logger.forces[self.frame_index, :3]
		hydro_forces = rotation_matrix @ self.robot.logger.hydro_forces[self.frame_index, :3]
  
		max_norm = np.max([np.linalg.norm(thrust_forces), np.linalg.norm(hydro_forces),np.linalg.norm(total_forces)])
		thrust_forces /= max_norm 
		total_forces /= max_norm
		hydro_forces /= max_norm
  
		glLineWidth(line_width)  # Set thicker line width
		glBegin(GL_LINES)
		# Thruster force
		glColor3f(1, 0.8, 0)
		glVertex3f(*robot_pos)
		glVertex3f(*(robot_pos + length * thrust_forces))
  
		# Total force
		glColor3f(1, 0, 1)
		glVertex3f(*robot_pos)
		glVertex3f(*(robot_pos + length * total_forces))

		# Hydro forces
		glColor3f(0, 1, 1)
		glVertex3f(*robot_pos)
		glVertex3f(*(robot_pos + length * hydro_forces))
		glEnd()
		glLineWidth(1.0)  # Reset line width
		
		if draw_on_top:
			glEnable(GL_DEPTH_TEST)  # Re-enable depth test
		
		glEnable(GL_LIGHTING)

	def draw_ground(self, size=100, step=1):
		glDisable(GL_LIGHTING)
		glBegin(GL_LINES)
		glColor3f(0.6, 0.6, 0.6)  # Less reflective grid
		for i in range(-size, size + 1, step):
			glVertex3f(i, -size, 0)
			glVertex3f(i, size, 0)
			glVertex3f(-size, i, 0)
			glVertex3f(size, i, 0)
		glEnd()
		glEnable(GL_LIGHTING)

	def display(self):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()

		if self.follow_robot and len(self.etas):
			tf = self.etas[self.frame_index]
			pos = tf[:3]
			self.pan_x, self.pan_y, self.pan_z = pos[0], pos[1], pos[2]

		cx = self.camera_radius * np.sin(self.camera_phi) * np.cos(self.camera_theta)
		cy = self.camera_radius * np.sin(self.camera_phi) * np.sin(self.camera_theta)
		cz = self.camera_radius * np.cos(self.camera_phi)

		gluLookAt(cx + self.pan_x, cy + self.pan_y, -cz + self.pan_z,
				  self.pan_x, self.pan_y, self.pan_z,
				  0, 0, -1)

		self.draw_ground()

		if self.gui.draw_robot_button.active:
			if len(self.etas):
				tf = self.etas[self.frame_index]
				glPushMatrix()
				glTranslatef(*tf[:3])
				glRotatef(tf[5] * RAD2DEG, 0.0, 0.0, 1.0)
				glRotatef(tf[4] * RAD2DEG, 0.0, 1.0, 0.0)
				glRotatef(tf[3] * RAD2DEG, 1.0, 0.0, 0.0)
				draw_vbo_textured(self.robot_vbo, self.robot_vertex_count, self.robot_texture_id)
				if self.gui.draw_reference_button.active:
					self.draw_axes()
				glPopMatrix()
    
		if self.gui.draw_robot_force_button.active:
			self.draw_robot_force(draw_on_top=True);
		
		if self.gui.draw_thruster_force_button.active:
			self.draw_thrusters_thrust(draw_on_top=True);

		if self.gui.draw_wps_button.active:
			if len(self.desired_tfs):
				for tf in self.desired_tfs:
					glPushMatrix()
					glTranslatef(*tf[:3])
					glRotatef(tf[5] * RAD2DEG, 0, 0, 1)
					glRotatef(tf[4] * RAD2DEG, 0, 1, 0)
					glRotatef(tf[3] * RAD2DEG, 1, 0, 0)
					self.draw_target_marker()
					if self.gui.draw_reference_button.active:
						self.draw_axes(draw_on_top=True)
					glPopMatrix()

		if self.gui.draw_trace_button.active:
			self.robot_trace.draw()
		if self.gui.draw_reference_button.active:
			self.draw_axes(0.5, 2, True)
		self.gui.draw(self.robot, self.fps, self.playback_speed, self.frame_index, self.dt)

		glutSwapBuffers()

	def reshape(self, w, h):
		new_width = w if w > 600 else 600
		new_height = h if h > 300 else 300
		self.window_width, self.window_height = new_width, new_height
		self.gui.update_window_size(new_width, new_height)

		glutReshapeWindow(new_width, new_height)
		glViewport(0, 0, new_width, new_height)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(60.0, new_width / new_height, 0.1, 1000.0)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()

	def zoom(self, direction):
		self.camera_radius *= 0.9 if direction > 0 else 1.1
		self.camera_radius = np.clip(self.camera_radius, 1.0, 1000.0)

	def keyboard(self, key, x, y):
		if key == b' ':
			self.running = not self.running
		elif key in (b's', b'S'):
			self.running = False
			self.step_request = True
		elif key in (b'r', b'R'):
			self.frame_index = 0
			self.frame_index_float = 0
		elif key in (b'f', b'F'):
			self.follow_robot = not self.follow_robot
		elif key in (b'g', b'G'):
			self.gui.draw_robot_force_button.active = not self.gui.draw_robot_force_button.active
		elif key in (b'h', b'H'):
			self.gui.draw_thruster_force_button.active = not self.gui.draw_thruster_force_button.active
		elif key in (b't', b'T'):
			self.gui.draw_trace_button.active = not self.gui.draw_trace_button.active
		elif key in (b'w', b'W'):
			self.gui.draw_wps_button.active = not self.gui.draw_wps_button.active
		elif key in (b'c', b'C'):
			self.gui.draw_reference_button.active = not self.gui.draw_reference_button.active
		elif key in (b'm', b'M'):
			self.gui.menu_button.active = not self.gui.menu_button.active
		elif key == b'\x1b':  # ESC
			print("Exiting simulation...")
			glutLeaveMainLoop()

	def special_keys(self, key, x, y):
		if key == GLUT_KEY_LEFT:
			self.playback_speed = np.sign(self.playback_speed) * max(1/2**6, np.abs(self.playback_speed) / 2)
		elif key == GLUT_KEY_DOWN:
			self.playback_speed = np.abs(self.playback_speed)
		elif key == GLUT_KEY_UP:
			self.playback_speed = -np.abs(self.playback_speed)
		elif key == GLUT_KEY_RIGHT:
			self.playback_speed = np.sign(self.playback_speed) * min(2**6, np.abs(self.playback_speed) * 2)

	def update(self, value):
		if not self.initialized:
			self.last_time = time.time()
			self.initialized = True

		self.current_time = time.time()
		if self.running:
			elapsed = self.current_time - self.fps_last_time
			step = self.playback_speed * (self.current_time - self.last_time) / self.dt
			self.frame_index_float += step
			self.frame_index = int(self.frame_index_float) % len(self.etas)

			self.frame_count += 1
			if elapsed >= 1:
				self.fps = self.frame_count / elapsed
				self.frame_count = 0
				self.fps_last_time = self.current_time

		if self.step_request:
			step = self.playback_speed
			self.frame_index_float += step
			self.frame_index = int(self.frame_index_float) % len(self.etas)
			self.step_request = False
		self.last_time = self.current_time

		if self.gui.draw_trace_button.active:
			self.robot_trace.update(self.etas[:, :3])


		glutPostRedisplay()
		glutTimerFunc(40, self.update, 0)

	def mouse(self, button, state, x, y):
		if state == GLUT_DOWN:
			self.mouse_button = button
			self.mouse_prev = [x, y]
		else:
			self.mouse_button = None

		y = self.window_height - y  # Invert Y for UI coords

		# Toggle menu button
		self.gui.menu_button.handle_mouse(state)
		
		if state != GLUT_DOWN:
			return
		
		if not self.gui.menu_button.active:
			return

		self.gui.draw_reference_button.handle_mouse(state)
		self.gui.draw_trace_button.handle_mouse(state)
		self.gui.draw_wps_button.handle_mouse(state)
		self.gui.draw_robot_button.handle_mouse(state)
		self.gui.draw_robot_force_button.handle_mouse(state)

	def hover_button(self, x, y):
		y = self.window_height - y  # Invert Y for UI coords
		self.gui.menu_button.handle_mouse_motion(x, y)

		if not self.gui.menu_button.active:
			return
		self.gui.draw_reference_button.handle_mouse_motion(x, y)
		self.gui.draw_trace_button.handle_mouse_motion(x, y)
		self.gui.draw_wps_button.handle_mouse_motion(x, y)
		self.gui.draw_robot_button.handle_mouse_motion(x, y)
		self.gui.draw_robot_force_button.handle_mouse_motion(x, y)


	def motion(self, x, y):
		dx = x - self.mouse_prev[0]
		dy = y - self.mouse_prev[1]
		self.mouse_prev = [x, y]

		if self.mouse_button == GLUT_RIGHT_BUTTON:
			self.camera_theta += dx * 0.005
			self.camera_phi -= dy * 0.005
			self.camera_phi = np.clip(self.camera_phi, 0.01, np.pi - 0.01)
		elif self.mouse_button == GLUT_MIDDLE_BUTTON:
			cam_x = self.camera_radius * np.sin(self.camera_phi) * np.cos(self.camera_theta)
			cam_y = self.camera_radius * np.sin(self.camera_phi) * np.sin(self.camera_theta)
			cam_z = self.camera_radius * np.cos(self.camera_phi)

			forward = np.array([-cam_x, -cam_y, cam_z])
			forward /= np.linalg.norm(forward)
			up = np.array([0, 0, -1])
			right = np.cross(forward, up)
			right /= np.linalg.norm(right)
			true_up = np.cross(right, forward)

			factor = 0.001 * self.camera_radius
			move = right * (-dx * factor) + true_up * (dy * factor)

			self.pan_x += move[0]
			self.pan_y += move[1]
			self.pan_z += move[2]

	def init_gl(self):
		# print(glGetString(GL_RENDERER).decode())
		# print(glGetString(GL_VENDOR).decode())
		# print(glGetString(GL_VERSION).decode())

		glClearColor(0.7, 0.7, 0.7, 1.0)
		glEnable(GL_DEPTH_TEST)

		# Lighting
		glEnable(GL_LIGHTING)
		glEnable(GL_LIGHT0)

		# Position light above and in front of scene
		light_position = [10.0, 10.0, 10.0, 1.0]  # w=1.0 means positional
		glLightfv(GL_LIGHT0, GL_POSITION, light_position)

		# Light color (white diffuse light)
		glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
		glLightfv(GL_LIGHT0, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0])

		# Enable color tracking
		glEnable(GL_COLOR_MATERIAL)
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

		# Optional: Add slight shininess
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 32)

		glMatrixMode(GL_PROJECTION)
		gluPerspective(60, self.window_width / self.window_height, 0.1, 1000.0)
		glMatrixMode(GL_MODELVIEW)

		
	def run(self):
		glutInit()
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
		glutInitWindowSize(self.window_width, self.window_height)
		glutCreateWindow(b"3D Robot Viewer")
		self.init_gl()

		glutReshapeFunc(self.reshape)
		glutDisplayFunc(self.display)
		glutTimerFunc(40, self.update, 0)
		glutMouseFunc(self.mouse)
		glutMotionFunc(self.motion)
		glutPassiveMotionFunc(self.hover_button)
		glutKeyboardFunc(self.keyboard)
		glutSpecialFunc(self.special_keys)

		try:
			glutMouseWheelFunc(lambda wheel, direction, x, y: self.zoom(direction))
		except Exception:
			print("Mouse wheel not supported; zoom with +/- keys instead.")

		self.robot.logger.np_format()
		self.load_sequences()
  
		# Load models
		robot_mesh = load_obj_with_tex("config/data/BlueROV2H.obj", "config/data/BlueROVTexture.png")
		vertex_data = create_vertex_data(*robot_mesh[:4])
		self.robot_vbo = create_vbo(vertex_data)
		self.robot_texture_id = robot_mesh[4]
		self.robot_vertex_count = len(vertex_data) // 8
		self.robot_trace = RobotTrace()

		# GUI
		self.gui = GUI(window_width=self.window_width, window_height=self.window_height)

		glutMainLoop()