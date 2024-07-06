class OBJ:
    def __init__(self, filename, swapyz=False, dx=0, dy=0, dz=0, marker_length=0.1, scale_factor=1.0):
        dx = dx * marker_length*1000
        dy = dy * marker_length*1000
        dz = dz * marker_length*1000
        """Loads a Wavefront OBJ file and applies optional displacement."""
        self.vertices = []
        self.normals = []
        self.texcoords = []
        self.faces = []
        
        for line in open(filename, "r"):
            if line.startswith('#'): continue
            values = line.split()
            if not values: continue
            if values[0] == 'v':
                v = list(map(float, values[1:4]))
                if swapyz:
                    v = v[0], v[2], v[1]
                v = [v[0] + dx, v[1] + dy, v[2] + dz]
                self.vertices.append(tuple(v))  # Convert back to tuple if needed
            elif values[0] == 'vn':
                v = list(map(float, values[1:4]))
                if swapyz:
                    v = v[0], v[2], v[1]
                self.normals.append(v)
            elif values[0] == 'vt':
                self.texcoords.append(list(map(float, values[1:3])))
            elif values[0] == 'f':
                face = []
                texcoords = []
                norms = []
                for v in values[1:]:
                    w = v.split('/')
                    face.append(int(w[0]))
                    if len(w) >= 2 and len(w[1]) > 0:
                        texcoords.append(int(w[1]))
                    else:
                        texcoords.append(0)
                    if len(w) >= 3 and len(w[2]) > 0:
                        norms.append(int(w[2]))
                    else:
                        norms.append(0)
                self.faces.append((face, norms, texcoords))
        
        self.scale_vertices(scale_factor)

    def print_vertices(self):
        for vertex in self.vertices:
            print(vertex)
        
        print("Total vertices: ", len(self.vertices))
    def print_normals(self):
        for normal in self.normals:
            print(normal)
        print("Total normals: ", len(self.normals))

    def print_texcoords(self):
        for texcoord in self.texcoords: # each texcoord is a map object, destructuring it to a list
            print(list(texcoord))
        print("Total texcoords: ", len(self.texcoords))

    def print_faces(self):
        for face in self.faces:
            print(face)
        print("Total faces: ", len(self.faces))

    def scale_vertices(self, scale_factor):
        scaled_vertices = []
        for vertex in self.vertices:
            scaled_vertex = [coord * scale_factor for coord in vertex]
            scaled_vertices.append(scaled_vertex)
        self.vertices = scaled_vertices

         # Scale normals if they exist
        if self.normals:
            scaled_normals = []
            for normal in self.normals:
                scaled_normal = [coord * scale_factor for coord in normal]
                scaled_normals.append(scaled_normal)
            self.normals = scaled_normals
        
        # Scale texcoords if they exist
        if self.texcoords:
            scaled_texcoords = []
            for texcoord in self.texcoords:
                scaled_texcoord = [coord * scale_factor for coord in texcoord]
                scaled_texcoords.append(scaled_texcoord)
            self.texcoords = scaled_texcoords