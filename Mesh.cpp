#include "Mesh.h"

using namespace std;

void Mesh::load( const char* filename )
{
	// 2.1.1. load() should populate bindVertices, currentVertices, and faces

	// Add your code here.
	ifstream myfile(filename);

    if (myfile.is_open()) {
		string buffer;

        while (getline(myfile, buffer)) {
            stringstream ss(buffer);

			string initial;
			ss >> initial;

			if (initial == "v") {
				Vector3f v;
				ss >> v[0] >> v[1] >> v[2];
				// cout << v[0] << " " << v[1] << " " << v[2] <<endl;
				bindVertices.push_back(v);
			} else if (initial == "f") {
				Tuple3u t;
				ss >> t[0] >> t[1] >> t[2];
				// cout << t[0] << " " << t[1] << " " << t[2] <<endl;
				faces.push_back(t);
			}
        }

        myfile.close();
	}

	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
}

// Inline functions to help with drawing
inline void glVertex( const Vector3f& a )
{
    glVertex3fv(a);
}

inline void glNormal( const Vector3f& a ) 
{
    glNormal3fv(a);
}

void Mesh::draw()
{
	// Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".
	
	glBegin(GL_TRIANGLES);
    for (unsigned i=0; i < faces.size(); i++)
    {
		// FACES INDEX FROM 1 NEVER FORGET THAT !!!
		Vector3f v0 = currentVertices[faces[i][0] - 1];
		Vector3f v1 = currentVertices[faces[i][1] - 1];
		Vector3f v2 = currentVertices[faces[i][2] - 1];

		// ALWAYS DRAW NORMALS BEFORE VERTICES !!!
		Vector3f n = Vector3f::cross(v1 - v0, v2 - v0).normalized();
		glNormal(n);

		glVertex(v0);
		glVertex(v1);
		glVertex(v2);
    }
    glEnd();
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 2.2. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments

	// numJoints INCLUDES THE ROOT !
	ifstream myfile(filename);

    if (myfile.is_open()) {
		string buffer;

        while (getline(myfile, buffer)) {
            stringstream ss(buffer);

			// Do not include weight of root. Always 0.
			vector<float> weights(numJoints - 1);
			for (int i = 0; i < numJoints - 1; i++) {
				ss >> weights[i];
			}

			attachments.push_back(weights);
        }

        myfile.close();
	}
}
