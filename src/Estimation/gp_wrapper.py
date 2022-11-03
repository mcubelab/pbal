import ctypes
import numpy as np



class gp_wrapper(object):
	def __init__(self,dll_fname='./build/libgp_python.so'):
		self.lib = ctypes.cdll.LoadLibrary(dll_fname)
		self.lib.init_gp.argtypes = [ctypes.c_double ,ctypes.c_double ,ctypes.c_double , ctypes.c_double , ctypes.c_double , ctypes.c_double , ctypes.c_double , ctypes.c_double ,ctypes.c_bool , ctypes.c_double]
		self.lib.init_gp.restype = ctypes.c_void_p

		self.lib.update_gp.argtypes = [ctypes.c_double,ctypes.c_double ,ctypes.c_double ,ctypes.c_double]
		self.lib.update_gp.restype = ctypes.c_void_p

		self.lib.update_contour.argtypes = []
		self.lib.update_contour.restype = ctypes.c_void_p

		self.lib.contour_length.argtypes = []
		self.lib.contour_length.restype = ctypes.c_int

		self.lib.eval_contour_x.argtypes = [ctypes.c_int]
		self.lib.eval_contour_x.restype = ctypes.c_double

		self.lib.eval_contour_y.argtypes = [ctypes.c_int]
		self.lib.eval_contour_y.restype = ctypes.c_double

		self.lib.evalAtPoint.argtypes = [ctypes.c_double,ctypes.c_double]
		self.lib.evalAtPoint.restype = ctypes.c_void_p

		self.lib.evalAtPoint_value.argtypes = []
		self.lib.evalAtPoint_value.restype = ctypes.c_double

		self.lib.evalAtPoint_dvdx.argtypes = []
		self.lib.evalAtPoint_dvdx.restype  = ctypes.c_double

		self.lib.evalAtPoint_dvdy.argtypes = []
		self.lib.evalAtPoint_dvdy.restype  = ctypes.c_double

	def init_gp(self,pvar,priorvar,testLim, testRes,isLocal, priorRad):
		self.lib.init_gp(pvar[0],pvar[1],pvar[2],priorvar[0],priorvar[1],priorvar[2],testLim, testRes,isLocal, priorRad)

	def update_gp(self,pos_vec,normal_vec):
		self.lib.update_gp(pos_vec[0],pos_vec[1],normal_vec[0],normal_vec[1])

	def evalAtPoint(self,x,y):
		self.lib.evalAtPoint(x,y)
		return np.array([self.lib.evalAtPoint_value(),self.lib.evalAtPoint_dvdx(),self.lib.evalAtPoint_dvdy()])

	def update_contour(self):
		self.lib.update_contour()

	def contour_length(self):
		return self.lib.contour_length()

	def eval_contour_x(self,i):
		return self.lib.eval_contour_x(i)
		
	def eval_contour_y(self,i):
		return self.lib.eval_contour_y(i)

	def eval_contour(self):
		self.update_contour()
		contour_len = self.contour_length()
		contour_x = np.array([0.0]*contour_len)
		contour_y = np.array([0.0]*contour_len)

		for i in range(contour_len):
			contour_x[i] = self.eval_contour_x(i)
			contour_y[i] = self.eval_contour_y(i)

		return contour_x,contour_y


