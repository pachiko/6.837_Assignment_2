#include "SkeletalModel.h"

#include <FL/Fl.H>

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);

	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(Matrix4f cameraMatrix, bool skeletonVisible)
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}

void SkeletalModel::loadSkeleton( const char* filename )
{
	// Load the skeleton from file here.
	ifstream myfile(filename);

    if (myfile.is_open()) {
		string buffer;

        while (getline(myfile, buffer)) {
            stringstream ss(buffer);

            Vector3f v; // translation
			ss >> v[0] >> v[1] >> v[2];

			int index; // -1 for root node
			ss >> index;

			Joint* joint = new Joint;
			joint->transform = Matrix4f::translation(v);

			if (index == -1) {
				m_rootJoint = joint;
			} else {
				m_joints[index]->children.push_back(joint);
			}

			m_joints.push_back(joint);
        }

        myfile.close();
	}
}

void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrixf() before your drawing call.

	recursiveDrawJoints(m_rootJoint);	
}

void SkeletalModel::recursiveDrawJoints(Joint* j) {
	// Push, Load, Draw, Recurse on children, Pop, Load
	m_matrixStack.push(j->transform);
	glLoadMatrixf(m_matrixStack.top());

	glutSolidSphere( 0.025f, 12, 12 );
	for (auto jj: j->children) {
		recursiveDrawJoints(jj);
	}

	m_matrixStack.pop();
	glLoadMatrixf(m_matrixStack.top());
}

void SkeletalModel::drawSkeleton( )
{
	// Draw boxes between the joints. You will need to add a recursive helper function to traverse the joint hierarchy.
	recursiveDrawSkeleton(m_rootJoint);	
}

void SkeletalModel::recursiveDrawSkeleton(Joint* j) {
	m_matrixStack.push(j->transform);
	glLoadMatrixf(m_matrixStack.top());

	for (auto jj : j->children) {
		// translate in z such that the box ranges from [0:5; 0:5; 0] to [0:5; 0:5; 1]
		Matrix4f cube = Matrix4f::translation(Vector3f(0.f, 0.f, 0.5f));

		Vector3f parentOffset = jj->transform.getCol(3).xyz(); // vector to child joint
		float l = parentOffset.abs(); // distance to a child joint
		
		// Remember, you start off from local coords: cube from [0:5; 0:5; 0] to [0:5; 0:5; 1]
		cube = Matrix4f::scaling(0.05f, 0.05f, l) * cube; // thin and long

		Vector3f rnd(0.f, 0.f, 1.f);
		Vector3f z = parentOffset.normalized();
		Vector3f y = Vector3f::cross(z, rnd).normalized();
		Vector3f x = Vector3f::cross(y, z).normalized();

		// Rotate z-axis to point to child
		Matrix4f rot(Vector4f(x, 0.f), Vector4f(y, 0.f), Vector4f(z, 0.f),
					Vector4f(Vector3f(), 1.f));
		cube = rot * cube;

		// Load matrix and draw cube
		m_matrixStack.push(cube);
		glLoadMatrixf(m_matrixStack.top());
		glutSolidCube( 1.0f );

		// Reset cube matrix and prepare for child's recursion
		m_matrixStack.pop();
		glLoadMatrixf(m_matrixStack.top());

		recursiveDrawSkeleton(jj);
	}

	m_matrixStack.pop();
	glLoadMatrixf(m_matrixStack.top());
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
	Matrix3f rot = Matrix3f::rotateX(rX) * Matrix3f::rotateY(rY) * Matrix3f::rotateZ(rZ);
	m_joints[jointIndex]->transform.setSubmatrix3x3(0, 0, rot);
}


void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.

	m_rootJoint->bindWorldToJointTransform = m_rootJoint->transform.inverse();
	recursiveComputeBindWorldToJointTransforms(m_rootJoint, m_rootJoint->bindWorldToJointTransform);
}

void SkeletalModel::recursiveComputeBindWorldToJointTransforms(Joint* j, Matrix4f parentT) {
	if (j != m_rootJoint) {
		j->bindWorldToJointTransform = j->transform.inverse()*parentT;
	}

	for (auto jj : j->children) {
		recursiveComputeBindWorldToJointTransforms(jj, j->bindWorldToJointTransform);
	}
}

void SkeletalModel::updateCurrentJointToWorldTransforms() {
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's currentJointToWorldTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.

	m_rootJoint->currentJointToWorldTransform = m_rootJoint->transform;
	recursiveUpdateCurrentJointToWorldTransforms(m_rootJoint, m_rootJoint->currentJointToWorldTransform);
}

void SkeletalModel::recursiveUpdateCurrentJointToWorldTransforms(Joint* j, Matrix4f parentT) {
	if (j != m_rootJoint) {
		j->currentJointToWorldTransform = parentT*j->transform;
	}

	for (auto jj : j->children) {
		recursiveUpdateCurrentJointToWorldTransforms(jj, j->currentJointToWorldTransform);
	}
}

void SkeletalModel::updateMesh() {
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.

	m_mesh.currentVertices.clear();

	for (size_t i = 0; i < m_mesh.bindVertices.size(); i++) {
		Vector4f newCV;
		const Vector4f bv = Vector4f(m_mesh.bindVertices[i], 1.f);
		vector<float> weights = m_mesh.attachments[i]; // weights vector for each vertex		

		for (size_t j = 0; j < weights.size(); j++) {
			float w = weights[j]; // J - 1, where J is the number of joints including the root joint
			Joint* jnt = m_joints[j + 1]; // Remember we do not store weights for root joint
			
			newCV = newCV + w // IT IS VITAL YOU DO NOT USE THE WEIGHTS TO SCALE THE TRANSFORMS. ONLY SCALE THE FINAL VECTOR
			*(jnt->currentJointToWorldTransform
				*jnt->bindWorldToJointTransform
				*bv);
		}
		
		m_mesh.currentVertices.push_back(newCV.xyz());
	}
}

