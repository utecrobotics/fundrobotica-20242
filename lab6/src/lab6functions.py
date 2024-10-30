import numpy as np
from copy import copy

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
 T1 = dh(0.52, q[0], 0.16, pi/2)
 T2 = dh(0, -q[1]+pi/2, 0.78, 0)
 T3 = dh(0, q[2], 0.15, pi/2)
 T4 = dh(0.86, q[3], 0, pi/2)
 T5 = dh(0, -q[4], 0, -pi/2)
 T6 = dh(0.153, q[5], 0, 0)
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

 J = np.zeros((7,6))
 # Implementar este Jacobiano aqui
 
    
 return J



def rot2quat(R):
 """
 Convertir una matriz de rotacion en un cuaternion

 Entrada:
   R -- Matriz de rotacion
 Salida:
   Q -- Cuaternion [ew, ex, ey, ez]
 """

 dEpsilon = 1e-6
 quat = 4*[0.,]

 quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
 if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
  quat[1] = 0.0
 else:
  quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
 if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
  quat[2] = 0.0
 else:
  quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
 if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
  quat[3] = 0.0
 else:
  quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

 return np.array(quat)



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
 
 quat = rot2quat(T[0:3,0:3])
 res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
 return np.array(res)



def skew(w):
 R = np.zeros([3,3])
 R[0,1] = -w[2]; R[0,2] = w[1]
 R[1,0] = w[2];  R[1,2] = -w[0]
 R[2,0] = -w[1]; R[2,1] = w[0]
 return R
