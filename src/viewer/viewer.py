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
DEG2RAD = 3.1415926535 / 180
class Viewer:
	def __init__(self, robot, timestep, window_width=1600, window_height=900):
		self.window_width = window_width
		self.window_height = window_height

		self.robot = robot
		self.dt = timestep

		# Playback & animation
		self.etas = None
		self.nus = None
		self.desired_etas = None
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
		self.bool_draw_hud = True


	def load_sequences(self):
		self.etas = self.robot.logger.etas
		self.nus = self.robot.logger.nus
		self.desired_etas = self.robot.logger.desired_etas
		self.times = self.robot.logger.timestamps

	def draw_target_marker(self, size=0.05):
		"""Draws a stylized 3D target marker with sphere and axis arrows."""
		# Draw central sphere
		glDisable(GL_LIGHTING)
		glColor3f(1.0, 0.9, 0.0)  # yellowish
		glutSolidSphere(size, 20, 20)
		glEnable(GL_LIGHTING)

	def draw_axes(self, length=0.5, line_width=1.5, draw_on_top=False):
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

	def draw_thrusters_thrust(self, length=1.0, line_width=1.5, draw_on_top=False):
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
			max_abs_thrust = np.max(np.abs(thrusters.thrust_limits[i]))
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

	def draw_hud(self):
		self.draw_robot_info_hud()
		self.draw_current_hud()
		# self.draw_ladders()
		self.draw_heading_ladder()
		self.draw_pitch_ladder()


	def draw_current_hud(self, radius=60):
		"""
		Draw a round HUD panel in the bottom-right showing the current vector.
		"""

		# ---- Retrieve current vector ----
		current = self.robot.logger.fluid_vels[self.frame_index, :3].copy()
		mag = np.linalg.norm(current)
		if mag > 1e-8:
			direction = current / mag
		else:
			direction = np.array([0, 0, 0], dtype=float)

		# ---- Screen size ----
		viewport = glGetIntegerv(GL_VIEWPORT)
		screen_w, screen_h = viewport[2], viewport[3]

		# ---- Switch to 2D mode ----
		glMatrixMode(GL_PROJECTION)
		glPushMatrix()
		glLoadIdentity()
		glOrtho(0, screen_w, 0, screen_h, -1, 1)

		glMatrixMode(GL_MODELVIEW)
		glPushMatrix()
		glLoadIdentity()

		glDisable(GL_LIGHTING)
		glDisable(GL_DEPTH_TEST)

		# --- Enable blending / line smoothing ---
		glEnable(GL_BLEND)
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
		glEnable(GL_LINE_SMOOTH)
		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)

		# ---- Panel center ----
		cx = screen_w - radius - 20
		cy = radius + 20

		# ------------------------------------------
		# Draw circular background
		# ------------------------------------------
		glColor4f(0.08, 0.08, 0.08, 0.7)
		glBegin(GL_TRIANGLE_FAN)
		glVertex2f(cx, cy)
		for i in range(0, 361, 3):
			ang = np.radians(i)
			glVertex2f(cx + np.cos(ang) * radius,
					cy + np.sin(ang) * radius)
		glEnd()

		# ------------------------------------------
		# Draw outline
		# ------------------------------------------
		glColor3f(0.25, 0.25, 0.25)
		glLineWidth(2)
		glBegin(GL_LINE_LOOP)
		for i in range(0, 361, 3):
			ang = np.radians(i)
			glVertex2f(cx + np.cos(ang) * radius,
					cy + np.sin(ang) * radius)
		glEnd()
		glLineWidth(1)

		# ------------------------------------------
		# Draw direction vector (arrow)
		# ------------------------------------------
		vec_length = radius * 0.75  # stay inside circle

		glColor3f(0.3, 0.6, 1.0)
		glLineWidth(2)
		glBegin(GL_LINES)
		glVertex2f(cx, cy)
		glVertex2f(cx + direction[0] * vec_length,
				cy + direction[1] * vec_length)
		glEnd()

		# ---- Draw arrow head ----
		if mag > 1e-8:
			end_x = cx + direction[0] * vec_length
			end_y = cy + direction[1] * vec_length

			left = np.array([-direction[1], direction[0]])
			right = -left

			ah = 10  # arrowhead size
			glBegin(GL_TRIANGLES)
			glVertex2f(end_x, end_y)
			glVertex2f(end_x + left[0] * ah,
					end_y + left[1] * ah)
			glVertex2f(end_x + right[0] * ah,
					end_y + right[1] * ah)
			glEnd()

		glLineWidth(1)

		# ------------------------------------------
		# Draw magnitude text under the circle
		# ------------------------------------------
		text = f"{mag:.2f}"

		glColor3f(1, 1, 1)
		glRasterPos2f(cx - 10, cy - radius - 15)

		for ch in text:
			glutBitmapCharacter(GLUT_BITMAP_8_BY_13, ord(ch))

		# ---- Restore state ----
		glEnable(GL_DEPTH_TEST)
		glEnable(GL_LIGHTING)

		glMatrixMode(GL_PROJECTION)
		glPopMatrix()
		glMatrixMode(GL_MODELVIEW)
		glPopMatrix()


	# ================================================================
	#      HEADING CYLINDER  —  rolling numbers on a curved drum
	# ================================================================
	def draw_heading_cylinder(self):
		"""
		Draw a rolling heading scale curved like a cylinder.
		heading: degrees [0..360)
		"""

		glMatrixMode(GL_PROJECTION)
		glPushMatrix()
		glLoadIdentity()
		gluOrtho2D(0,1,0,1)

		glMatrixMode(GL_MODELVIEW)
		glPushMatrix()
		glLoadIdentity()
		glDisable(GL_DEPTH_TEST)

		heading = self.etas[self.frame_index][5] * RAD2DEG
		self.ratio = self.window_width / self.window_height
		
		# Tape parameters
		tick_step = 5             # tick every 10°
		label_step = 15            # label every 30°

		glColor3f(0.05, 0.05, 0.05)
		glLineWidth(2)

		glPushMatrix()

		# CYLINDER PROJECTION PARAMETERS
		# CYLINDER CENTERED AT (0,0)
		cx = 0.0          # fixed center X
		cy = 0.0          # fixed center Y
		R  = 0.075         # radius of circle / cylinder

		for deg in range(-180 + tick_step, 181, tick_step):

			# Convert tape degree to arc angle around circle
			theta = (deg - heading + 45) * DEG2RAD

			tick_len  = 0.01
			label_len = 0.017

			# POSITION ON THE CIRCLE
			x = cx + R * np.sin(theta)
			y = cy + R * self.ratio * np.cos(theta)
			xt = cx + (R + tick_len) * np.sin(theta)
			yt = cy + (R + tick_len) * self.ratio * np.cos(theta)
			xl = cx + (R + label_len) * np.sin(theta)
			yl = cy + (R + label_len) * self.ratio * np.cos(theta)

			# ---- TICK ----
			glBegin(GL_LINES)
			glVertex2f(x, y)
			glVertex2f(xt, yt)
			glEnd()

			# ---- LABELS ----
			if deg % label_step == 0:
				glBegin(GL_LINES)
				glVertex2f(x, y)
				glVertex2f(xl, yl)
				glEnd()

				# label position slightly outside circle
				lx = cx + (R - 0.015) * np.sin(theta) 
				ly = cy + (R - 0.015) * np.cos(theta) * self.ratio

				self.draw_text(lx, ly, f"{deg:g}", font=GLUT_BITMAP_8_BY_13, color=(0.05, 0.05, 0.05))


		glPopMatrix()

		offset = 0.005
		# Draw center marker
		glColor3f(0.3, 0.8, 1.0)
		glBegin(GL_TRIANGLES)
		glVertex2f(cx + R - offset, (cy + R - offset) * self.ratio)
		glVertex2f(cx + R - offset + 0.005, (cy + R - offset + 0.015) * self.ratio)
		glVertex2f(cx + R - offset + 0.015, (cy + R - offset + 0.005) * self.ratio)
		glEnd()

		self.draw_text(R + 1.5 * offset, (R + 1.5 * offset) * self.ratio, f"{int(heading):g}°", font=GLUT_BITMAP_8_BY_13, color=(0.05, 0.05, 0.05))


		glLineWidth(1)

		# ---- Restore state ----
		glEnable(GL_DEPTH_TEST)
		glEnable(GL_LIGHTING)

		glMatrixMode(GL_PROJECTION)
		glPopMatrix()
		glMatrixMode(GL_MODELVIEW)
		glPopMatrix()



	# ================================================================
	#      HEADING CYLINDER  —  rolling numbers on a curved drum
	# ================================================================
	def draw_heading_ladder(self):
		"""
		Draw a rolling heading scale curved like a cylinder.
		heading: degrees [0..360)
		"""
		
		if np.isnan(self.etas[self.frame_index][5]):
			return
		
		glMatrixMode(GL_PROJECTION)
		glPushMatrix()
		glLoadIdentity()
		gluOrtho2D(0,1,0,1)

		glMatrixMode(GL_MODELVIEW)
		glPushMatrix()
		glLoadIdentity()
		glDisable(GL_DEPTH_TEST)


		heading = self.etas[self.frame_index][5] * RAD2DEG % 360
		heading = heading * (abs(heading) <= 180) + (heading - 360) * (heading > 180) + (heading + 360) * (heading < -180)
		self.ratio = self.window_width / self.window_height

		
		# Tape parameters
		tick_step = 5             # tick every 10°
		label_step = 15            # label every 30°

		glColor3f(0.05, 0.05, 0.05)
		glLineWidth(2)

		glPushMatrix()

		# CYLINDER PROJECTION PARAMETERS
		# CYLINDER CENTERED AT (0,0)
		cx = 0.16          # fixed center X
		cy = 0.025          # fixed center Y
		cyl = 0.01
		width = 0.3
		angle_span = 120

		for deg in range(int(-180 + tick_step - angle_span / 2), int(181 + angle_span / 2), tick_step):
			
			r = (deg - heading) / 180
			if abs(r) > angle_span / 2 / 180:
				continue
			
			r *= width

			tick_len  = 0.01
			label_len = 0.017

			# POSITION ON THE CIRCLE
			x = cx + r
			y = cy 
			xt = cx + r
			yt = cy + tick_len
			xl = cx + r
			yl = cy + label_len

			# ---- TICK ----
			glBegin(GL_LINES)
			glVertex2f(x, y)
			glVertex2f(xt, yt)
			glEnd()

			# ---- LABELS ----
			if deg % label_step == 0:
				glBegin(GL_LINES)
				glVertex2f(x, y)
				glVertex2f(xl, yl)
				glEnd()

				# label position slightly outside circle
				lx = cx + r
				ly = cyl
				deg_label = f"{deg * (abs(deg) <= 180) + (deg - 360) * (deg > 180) + (deg + 360) * (deg < -180):g}"
				label_size = 8 / self.window_width * len(deg_label)
				self.draw_text(lx - label_size / 2, ly, deg_label, font=GLUT_BITMAP_8_BY_13, color=(0.05, 0.05, 0.05))


		glPopMatrix()

		offset = 0.005
		# Draw center marker
		glColor3f(0.3, 0.8, 1.0)
		glBegin(GL_TRIANGLES)
		glVertex2f(cx, cy + 0.02)
		glVertex2f(cx - 0.005, cy + 0.02 + 0.015)
		glVertex2f(cx + 0.005, cy + 0.02 + 0.015)
		glEnd()
		heading_label = f"{int(heading):g}°"
		label_size = 8 / self.window_width * (len(heading_label) - 1)
		self.draw_text(cx - label_size / 2, cy + 0.02 + 0.02, heading_label, font=GLUT_BITMAP_8_BY_13, color=(0.05, 0.05, 0.05))


		glLineWidth(1)

		# ---- Restore state ----
		glEnable(GL_DEPTH_TEST)
		glEnable(GL_LIGHTING)

		glMatrixMode(GL_PROJECTION)
		glPopMatrix()
		glMatrixMode(GL_MODELVIEW)
		glPopMatrix()




	# ================================================================
	#      HEADING CYLINDER  —  rolling numbers on a curved drum
	# ================================================================
	def draw_pitch_ladder(self):
		"""
		Draw a rolling heading scale curved like a cylinder.
		heading: degrees [0..360)
		"""
		if np.isnan(self.etas[self.frame_index][4]):
			return

		glMatrixMode(GL_PROJECTION)
		glPushMatrix()
		glLoadIdentity()
		gluOrtho2D(0,1,0,1)

		glMatrixMode(GL_MODELVIEW)
		glPushMatrix()
		glLoadIdentity()
		glDisable(GL_DEPTH_TEST)


		pitch = self.etas[self.frame_index][4] * RAD2DEG
		pitch = pitch * (abs(pitch) <= 90) + (pitch - 180) * (pitch > 90) + (pitch + 180) * (pitch < -90)

		self.ratio = self.window_width / self.window_height
		
		# Tape parameters
		tick_step = 5             # tick every 10°
		label_step = 15            # label every 30°

		glColor3f(0.05, 0.05, 0.05)
		glLineWidth(2)

		glPushMatrix()

		# CYLINDER PROJECTION PARAMETERS
		# CYLINDER CENTERED AT (0,0)
		cx = 0.025          # fixed center X
		cy = 0.20        # fixed center Y
		cxl = 0.01
		height = 0.3
		angle_span = 90
		tick_len  = 0.01 / self.ratio
		label_len = 0.017 / self.ratio

		for deg in range(int(-90 + tick_step - angle_span / 2), int(90 + angle_span / 2), tick_step):
			
			r = (deg - pitch) / 180
			if abs(r) > angle_span / 2 / 180:
				continue
			
			r *= height


			# POSITION ON THE CIRCLE
			x = cx 
			y = cy + r * self.ratio
			xt = cx + tick_len
			yt = cy + r * self.ratio
			xl = cx + label_len
			yl = cy + r * self.ratio

			# ---- TICK ----
			glBegin(GL_LINES)
			glVertex2f(x, y)
			glVertex2f(xt, yt)
			glEnd()

			# ---- LABELS ----
			if deg % label_step == 0:
				glBegin(GL_LINES)
				glVertex2f(x, y)
				glVertex2f(xl, yl)
				glEnd()

				# label position slightly outside circle
				lx = cxl
				ly = cy + r * self.ratio
				deg_label = f"{deg * (abs(deg) <= 90) + (deg - 180) * (deg > 90) + (deg + 180) * (deg < -90):g}"
				label_size = 8 / self.window_width * len(deg_label)
				self.draw_text(lx, ly - label_size / 2, deg_label, font=GLUT_BITMAP_8_BY_13, color=(0.05, 0.05, 0.05))


		glPopMatrix()

		offset = 0.005
		# Draw center marker
		glColor3f(0.3, 0.8, 1.0)
		glBegin(GL_TRIANGLES)
		glVertex2f(cx + 0.015, cy)
		glVertex2f(cx + 0.015 + 0.01, cy - 0.005 * self.ratio)
		glVertex2f(cx + 0.015 + 0.01, cy + 0.005 * self.ratio)
		glEnd()
		pitch_label = f"{int(pitch):g}°"
		label_size = 8 / self.window_width * (len(pitch_label) - 1)
		self.draw_text(cx + 0.015 + 0.015, cy - label_size / 2, pitch_label, font=GLUT_BITMAP_8_BY_13, color=(0.05, 0.05, 0.05))


		glLineWidth(1)

		# ---- Restore state ----
		glEnable(GL_DEPTH_TEST)
		glEnable(GL_LIGHTING)

		glMatrixMode(GL_PROJECTION)
		glPopMatrix()
		glMatrixMode(GL_MODELVIEW)
		glPopMatrix()


	def draw_text(self, x, y, text, font=GLUT_BITMAP_9_BY_15, color=(1, 1, 1)):
		glColor3f(*color)
		glRasterPos2f(x, y)
		for ch in text:
			glutBitmapCharacter(font, ord(ch))

	def draw_robot_info_hud(self, line_height=20, panel_width=300, panel_padding=15, border_thickness=2):
		"""
		Draw a cockpit-style HUD for the robot.
		Uses self.etas[self.frame_index] and self.nus[self.frame_index].
		"""
		eta = self.etas[self.frame_index]
		nu = self.nus[self.frame_index]

		# --- Save OpenGL state ---
		glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_LINE_BIT)
		glDisable(GL_DEPTH_TEST)
		glDisable(GL_LIGHTING)

		# --- Save matrices ---
		glMatrixMode(GL_PROJECTION)
		glPushMatrix()
		glLoadIdentity()

		glMatrixMode(GL_MODELVIEW)
		glPushMatrix()
		glLoadIdentity()

		# --- Setup orthographic projection ---
		viewport = glGetIntegerv(GL_VIEWPORT)
		width, height = viewport[2], viewport[3]
		glOrtho(0, width, 0, height, -1, 1)

		# --- Enable blending / line smoothing ---
		glEnable(GL_BLEND)
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
		glEnable(GL_LINE_SMOOTH)
		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)

		# --- Flip Y-axis so (0,0) is top-left ---
		glTranslatef(0, height, 0)
		glScalef(1, -1, 1)

		# --- Panel position: center-bottom ---
		panel_height = line_height + 2 * panel_padding
		panel_x = (width - panel_width) / 2
		panel_y = height - panel_height - 30  # margin from bottom

		# --- Draw rounded rectangle panel (approx) ---
		# For simplicity, use multiple small line segments for rounded corners
		corner_radius = 10
		segments = 16

		def draw_rounded_rect(x, y, w, h, r):
			glBegin(GL_POLYGON)
			# Bottom-left corner
			for i in range(segments + 1):
				theta = np.pi + (np.pi / 2) * (i / segments)
				glVertex2f(x + r + r * np.cos(theta), y + r + r * np.sin(theta))
			# Bottom-right
			for i in range(segments + 1):
				theta = 1.5 * np.pi + (np.pi / 2) * (i / segments)
				glVertex2f(x + w - r + r * np.cos(theta), y + r + r * np.sin(theta))
			# Top-right
			for i in range(segments + 1):
				theta = 0 + (np.pi / 2) * (i / segments)
				glVertex2f(x + w - r + r * np.cos(theta), y + h - r + r * np.sin(theta))
			# Top-left
			for i in range(segments + 1):
				theta = 0.5 * np.pi + (np.pi / 2) * (i / segments)
				glVertex2f(x + r + r * np.cos(theta), y + h - r + r * np.sin(theta))
			glEnd()

		# Panel background
		# glColor4f(0.08, 0.08, 0.08, 0.7)
		glColor3f(0.8, 0.8, 0.8)
		glLineWidth(border_thickness)
		draw_rounded_rect(panel_x, panel_y, panel_width, panel_height, corner_radius)
		glLineWidth(1)

		# Panel border
		glColor4f(0.08, 0.08, 0.08, 0.7)
		draw_rounded_rect(panel_x, panel_y, panel_width, panel_height, corner_radius)


		# --- Draw speed bar ---
		speed = np.linalg.norm(nu[:3])
		max_speed = 1.5  # adjust as needed
		bar_width = panel_width - 2 * panel_padding
		bar_height = 10
		bar_x = panel_x + panel_padding
		bar_y = panel_y + panel_padding

		# Background
		glColor3f(0.2, 0.2, 0.2)
		glBegin(GL_QUADS)
		glVertex2f(bar_x, bar_y)
		glVertex2f(bar_x + bar_width, bar_y)
		glVertex2f(bar_x + bar_width, bar_y + bar_height)
		glVertex2f(bar_x, bar_y + bar_height)
		glEnd()

		# Filled speed
		filled_width = min(bar_width, bar_width * (speed / max_speed))
		if speed > max_speed:
			glColor3f(0.9, 0.3, 0.2)
		else:
			glColor3f(0.3, 0.8, 1.0)
	
		glBegin(GL_QUADS)
		glVertex2f(bar_x, bar_y)
		glVertex2f(bar_x + filled_width, bar_y)
		glVertex2f(bar_x + filled_width, bar_y + bar_height)
		glVertex2f(bar_x, bar_y + bar_height)
		glEnd()

		# --- Draw speed text ---
		self.draw_text(bar_x + bar_width / 2 - 20, bar_y + line_height * 1.5, f"{speed:.2f} m/s", color=(0.9, 0.9, 0.9))

		# --- Restore matrices and state ---
		glMatrixMode(GL_MODELVIEW)
		glPopMatrix()
		glMatrixMode(GL_PROJECTION)
		glPopMatrix()
		glPopAttrib()


	def draw_robot_force(self, length=0.5, line_width=1.5, draw_on_top=False):
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
			self.draw_robot_force(draw_on_top=True)

		if self.bool_draw_hud:
			self.draw_hud()

		
		if self.gui.draw_thruster_force_button.active:
			self.draw_thrusters_thrust(draw_on_top=True)

		if self.gui.draw_wps_button.active:
			if len(self.desired_etas):
				for tf in self.desired_etas:
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
		elif key in (b'v', b'V'):
			self.gui.draw_thruster_force_button.active = not self.gui.draw_thruster_force_button.active
		elif key in (b'h', b'H'):
			self.bool_draw_hud = not self.bool_draw_hud
		elif key in (b't', b'T'):
			self.gui.draw_trace_button.active = not self.gui.draw_trace_button.active
		elif key in (b'w', b'W'):
			self.gui.draw_wps_button.active = not self.gui.draw_wps_button.active
		elif key in (b'c', b'C'):
			self.gui.draw_reference_button.active = not self.gui.draw_reference_button.active
		elif key in (b'm', b'M'):
			self.gui.menu_button.active = not self.gui.menu_button.active
		elif key in (b'l', b'L'):
			self.camera_phi = 0.0001
			self.camera_theta = np.pi
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
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE)
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