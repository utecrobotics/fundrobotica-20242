import numpy as np
from copy import copy
from pyquaternion import Quaternion

pi = np.pi


def dh(d, theta, a, alpha):
 """
 Calcular la matriz de transformacion homogenea asociada con los parametros
 de Denavit-Hartenberg.
 Los valores d, theta, a, alpha son escalares.
 """
 sth = np.sin(theta)
 cth = np.cos(theta)
 sa  = np.sin(alpha)
 ca  = np.cos(alpha)
 T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
               [sth,  ca*cth, -sa*cth, a*sth],
               [0.0,      sa,      ca,     d],
               [0.0,     0.0,     0.0,   1.0]])
 return T



def fkine(q):
 """
 Calcular la cinematica directa del brazo robotico dados sus valores articulares. 
 q es un vector numpy de la forma [q1, q2, q3, ..., qn]
 """
 # Matrices DH (completar)
 T1 = dh(0.08920, q[0], 0, pi/2)
 T2 = dh(0, q[1], -0.425, 0)
 T3 = dh(0, q[2], -0.392, 0)
 T4 = dh(0.10930, q[3]+pi, 0, -pi/2)
 T5 = dh(0.09475, q[4], 0, pi/2)
 T6 = dh(0.08250, q[5], 0, 0)
 # Efector final con respecto a la base
 T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
 return T



def jacobian(q, delta=0.0001):
 """
 Jacobiano analitico para la posicion de un brazo robotico de n grados de libertad. 
 Retorna una matriz de 3xn y toma como entrada el vector de configuracion articular 
 q=[q1, q2, q3, ..., qn]
 """
 # Crear una matriz 3xn
 n = q.size
 J = np.zeros((3,n))
 # Calcular la transformacion homogenea inicial (usando q)
 T = fkine(q)
 # Iteracion para la derivada de cada articulacion (columna)
 for i in range(n):
  # Copiar la configuracion articular inicial
  dq = copy(q)
  # Calcular nuevamenta la transformacion homogenea e
  # Incrementar la articulacion i-esima usando un delta
  dq[i] += delta
  # Transformacion homogenea luego del incremento (q+delta)
  T_inc = fkine(dq)
  # Aproximacion del Jacobiano de posicion usando diferencias finitas
  J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
 return J



def jacobian_pose(q, delta=0.0001):
 """
 Jacobiano analitico para la posicion y orientacion (usando un
 cuaternion). Retorna una matriz de 7xn y toma como entrada el vector de
 configuracion articular q=[q1, q2, q3, ..., qn]
 """
 n = q.size
 J = np.zeros((7,n))
 # Implementar este Jacobiano aqui
 
    
 return J



def TF2xyzquat(T):
 """
 Convert a homogeneous transformation matrix into the a vector containing the
 pose of the robot.

 Input:
   T -- A homogeneous transformation
 Output:
   X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
 """
 quat = Quaternion(matrix=T[0:3,0:3])
 return np.array([T[0,3], T[1,3], T[2,3], quat.w, quat.x, quat.y, quat.z])



def PoseError(x,xd):
 """
 Determine the pose error of the end effector.

 Input:
 x -- Actual position of the end effector, in the format [x y z ew ex ey ez]
 xd -- Desire position of the end effector, in the format [x y z ew ex ey ez]
 Output:
 err_pose -- Error position of the end effector, in the format [x y z ew ex ey ez]
 """
 pos_err = x[0:3]-xd[0:3]
 qact = Quaternion(x[3:7])
 qdes = Quaternion(xd[3:7])
 qdif =  qdes*qact.inverse
 qua_err = np.array([qdif.w,qdif.x,qdif.y,qdif.z])
 err_pose = np.hstack((pos_err,qua_err))
 return err_pose
