import skimage.measure
import skimage.morphology
import visvis as vv
from visvis.utils import iso
import numpy as np



def ball(radius, dtype=np.uint8):
    # New from skimage.morphology
    n = 2 * radius + 1
    Z, Y, X = np.mgrid[ -radius:radius:n*1j, 
                        -radius:radius:n*1j, 
                        -radius:radius:n*1j]
    s = X**2 + Y**2 + Z**2
    return np.array(s <= radius * radius, dtype=dtype)


# Load volume
vol = ball(100, np.float32)
#vol = vv.volread('stent')

level = 0.5  # 0.5 for ball, 500 for stent

# Get mesh
vertices, faces = skimage.measure.marching_cubes(vol, level)
vertices, faces = np.array(vertices, np.float32), np.array(faces, np.int32)
vertices2  = vertices.copy()
# Correct for dimension ordering bug
vertices2[:,0]= vertices[:,2]
vertices2[:,1]= vertices[:,1]
vertices2[:,2]= vertices[:,0]
   
# Turn into visvis mesh object
bm1 = vv.BaseMesh(vertices2, faces)
#vv.processing.unwindFaces(bm1)  # This removes the artifacts, but results in a blocky appearance

# Create mesh using Lewiner method
bm2 = iso.isosurface(vol, level)
bm2._normals = None  # For better comparison, let visvis calculate the normals from the vertices+faces

# Visualise
vv.figure(1); vv.clf()
a1 = vv.subplot(121)
vv.mesh(bm1)
a2 = vv.subplot(122)
vv.mesh(bm2)
a1.camera = a2.camera