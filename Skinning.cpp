#include <math.h>
#include <stdio.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include <map>

#include <GL/glut.h>

#define GLUT_KEY_ESCAPE 27
#ifndef GLUT_WHEEL_UP
#define GLUT_WHEEL_UP 3
#define GLUT_WHEEL_DOWN 4
#endif


#include "Vector.h"
#include "Matrix.h"
#include "Geometry.h"
#include "Skeleton.h"
#include "Animation.h"
#include "Camera.h"

#include <sstream>

//mouse interface, camera
static const int WIDTH = 800;
static const int HEIGHT = 600;

static int mouse_left_pressed = 0;
static int mouse_right_pressed = 0;

static int last_x = -1;
static int last_y = -1;

static Camera* camera = NULL;

static Mesh* character = NULL;

static Animation* rest_animation = NULL;
static Animation* run_animation = NULL;
static Animation* walk_animation = NULL;

static Animation* current_animation = NULL;

/*gb - global frame
 *lc - local frame
 *tpf - transforms per frame
 *we need  local only for rest pose
 */
static std::vector<Matrix_4x4> rest_trans_lc;
static std::vector<std::vector<Matrix_4x4> > rest_tpf_gb;
static std::vector<std::vector<Matrix_4x4> > run_tpf_gb;
static std::vector<std::vector<Matrix_4x4> > walk_tpf_gb;
static std::vector<std::vector<Matrix_4x4> > current_tpf_gb;

/* This timer can be used to cycle through the frames of animation */
static float timer = 0;
static float timer_glut = 0;

/*variables to control the workflow */
/*display variables */
static std::string hint="Default Mode";
static bool show_skeleton = false;
static bool show_mesh = true;
static bool interpolate = false;

/*speed of animation*/
static float frames_per_second = 30.0f;
static float max_frames_per_second = 30.0f;
static float min_frames_per_second = 1.0f;

/* to keep animation speed within certain limit */
static float Clamp(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}

static std::string Int2String(int n) {
    std::ostringstream oss;
    oss << n;
    return oss.str();
}

//Normalise weights components to one, otherwise strange artifacts
static Vector3 NormSumToOne(Vector3 v) {
	//assumes that all components of v are positive;
	float sum = v.x + v.y + v.z;
	return Vector3(v.x/sum, v.y/sum, v.z/sum);
}

//must be called after rest_trans_lc has been initialised
static Vertex LinearBlending(Vertex& original, std::vector<Matrix_4x4>& anim_trans_gb) {
	Vector3 pos = Vector3::Zero();
	Vector3 norm = Vector3::Zero();
	Vector3 weight_amounts = NormSumToOne(original.weight_amounts);
	for (int j = 0; j < 3; j++) {
		int joint_id = (int)round(original.weight_ids[j]);
		float weight = weight_amounts[j];
		Matrix_4x4 trans =  anim_trans_gb[joint_id] * rest_trans_lc[joint_id];
		Matrix_3x3 rot = Matrix_4x4::ToMatrix_3x3(trans);
		pos += (trans * original.position * weight);
		norm += (rot * original.normal * weight);
	}
	return Vertex(pos, norm);
}

/*
 * Global transforms per frame for given animation.
 * Using vectors to alleviate problems with memory leaks.
 * Using this signature because it is c++98 and in that it won't be copying values twice as
 * c++98 doesn't have move constructor.
 */
static void ComputeTransPerFrameGB(std::vector<std::vector<Matrix_4x4> >& trans_per_frame_gb, Animation* anim) {
	trans_per_frame_gb.resize(anim->NumFrames());
	for (int frame_id = 0; frame_id < anim->NumFrames(); frame_id++) {
		Skeleton* skel = anim->GetFrame(frame_id);
		trans_per_frame_gb[frame_id].resize(skel->NumJoints());
		for (int joint_id = 0; joint_id < skel->NumJoints(); joint_id++) {
			trans_per_frame_gb[frame_id][joint_id] = skel->JointTransform(joint_id);
		}
	}
}

/*
 * Compute local transforms for Rest pose
 */
static void ComputeRestTransLC(std::vector<Matrix_4x4>& rest_trans_lc, Skeleton* rest_skel) {
	rest_trans_lc.resize(rest_skel->NumJoints());
	for (int joint_id = 0; joint_id < rest_skel->NumJoints(); joint_id++) {
		rest_trans_lc[joint_id] = Matrix_4x4::Inverse(rest_skel->JointTransform(joint_id));
	}
}

void DrawTextHint() {
	//relying on http://stackoverflow.com/questions/20866508/using-glut-to-simply-print-text
	glMatrixMode( GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, WIDTH, 0, HEIGHT);
	glMatrixMode( GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	//blue color
	glColor4f(0.0, 0.0, 1.0, 1.0);
	glRasterPos2i(10, 10); // move in 10 pixels from the left and bottom edges
	for (size_t i = 0; i < hint.length(); ++i) {
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, hint[i]);
	}
	glPopMatrix();
	glMatrixMode( GL_PROJECTION);
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW);
}


void Update() {
    timer += 0.05;
    //time since start (glutGet(GLUT_ELAPSED_TIME) gives milliseconds, so divide by 1000)
    timer_glut = glutGet(GLUT_ELAPSED_TIME)/1000.f;
    //std::ostringstream oss;
    //oss << timer_glut;
    //hint = oss.str();
    glutPostRedisplay();
}

static void DrawAxis(Matrix_4x4 origin) {

    const float size = 0.5;

    Vector3 center = origin * Vector3::Zero();
    Vector3 axis_x = origin * Vector3(size,0,0);
    Vector3 axis_y = origin * Vector3(0,size,0);
    Vector3 axis_z = origin * Vector3(0,0,size);

    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor4f(1.0, 0.0, 0.0, 1.0);
    glVertex3f(center.x, center.y, center.z);
    glVertex3f(axis_x.x, axis_x.y, axis_x.z);

    glColor4f(0.0, 1.0, 0.0, 1.0);
    glVertex3f(center.x, center.y, center.z);
    glVertex3f(axis_y.x, axis_y.y, axis_y.z);

    glColor4f(0.0, 0.0, 1.0, 1.0);
    glVertex3f(center.x, center.y, center.z);
    glVertex3f(axis_z.x, axis_z.y, axis_z.z);

    glEnd();
    glLineWidth(1.0f);
}

static void DrawSkeleton(Skeleton* skeleton) {

    glColor4f(0.0, 0.0, 0.0, 1.0);
    glLineWidth(2.0f);

    glBegin(GL_LINES);

    for (int i = 0; i < skeleton->NumJoints(); i++) {
        int bone_id = i;
        int parent_id = skeleton->GetJoint(bone_id).parent_id;

        if (parent_id == -1) continue;

        Vector3 bone_pos = skeleton->JointTransform(bone_id) * Vector3::Zero();
        Vector3 parent_pos = skeleton->JointTransform(parent_id) * Vector3::Zero();

        glVertex3f(bone_pos.x, bone_pos.y, bone_pos.z);
        glVertex3f(parent_pos.x, parent_pos.y, parent_pos.z);
    }

    glEnd();

    glLineWidth(1.0f);
    glColor4f(1.0, 1.0, 1.0, 1.0);

    for(int i = 0; i < skeleton->NumJoints(); i++) {
        DrawAxis(skeleton->JointTransform(i));
    }
}

// t - interpolation parameter between 0 and 1
static void DrawSkeletonInterpolated(Skeleton* skel_1, Skeleton* skel_2, float t) {
	if (skel_1->NumJoints() != skel_2->NumJoints()) {
		printf("Number of joints in the skeletons doesn't match!");
		return;
	}
    glColor4f(0.0, 0.0, 0.0, 1.0);
    glLineWidth(2.0f);

    glBegin(GL_LINES);

    for (int i = 0; i < skel_1->NumJoints(); i++) {
        int parent_id1 = skel_1->GetJoint(i).parent_id;
        if (parent_id1 == -1) continue;

        Vector3 bone_pos1 = skel_1->JointTransform(i) * Vector3::Zero();
        Vector3 parent_pos1 = skel_1->JointTransform(parent_id1) * Vector3::Zero();

        //second
        int parent_id2 = skel_2->GetJoint(i).parent_id;
        if (parent_id2 == -1) continue;

        Vector3 bone_pos2 = skel_2->JointTransform(i) * Vector3::Zero();
        Vector3 parent_pos2 = skel_2->JointTransform(parent_id2) * Vector3::Zero();

        //interpolation

        Vector3 bone_pos = bone_pos1 * (1-t) + bone_pos2 * t;
        Vector3 parent_pos = parent_pos1 * (1-t) + parent_pos2 * t;
        glVertex3f(bone_pos.x, bone_pos.y, bone_pos.z);
        glVertex3f(parent_pos.x, parent_pos.y, parent_pos.z);
    }

    glEnd();

    glLineWidth(1.0f);
    glColor4f(1.0, 1.0, 1.0, 1.0);
}

static void DrawModel() {

    float* world_positions_array = new float[character->NumVertices() * 3];
    float* world_normals_array = new float[character->NumVertices() * 3];
    int* triangle_array = new int[character->NumTriangles() * 3];


    /*
    ** TODO: Uncomment this once `JointTransform` is implemented to draw
    **       the skeleton of the character in the rest pose.
    */
    int curr_anim_frame = int(timer_glut * frames_per_second) % current_animation->NumFrames();
    //time interpolation parameter between 0 and 1
	float t = timer_glut * frames_per_second - int(timer_glut * frames_per_second);
	t = Clamp(t, 0, 1);
	int next_anim_frame = (curr_anim_frame + 1) % current_animation->NumFrames();
    //VISUALIZATION PART -----------------------------------------------------------------

    if (show_skeleton) {
    	if (interpolate) {
    		DrawSkeletonInterpolated(current_animation->GetFrame(curr_anim_frame), current_animation->GetFrame(next_anim_frame), t);
    	} else {
    		DrawSkeleton(current_animation->GetFrame(curr_anim_frame));
    	}
    }

    if (show_mesh) {

		for (int i = 0; i < character->NumVertices(); i++) {
			/*
			** TODO: Transform position and normal using rest pose and some
			**       other animated pose from one of the animations
			*/
			Vertex vrtx_original = character->GetVertex(i);
			Vertex vrtx;
			if (interpolate) {
				Vertex vrtx_1 = LinearBlending(vrtx_original, current_tpf_gb[curr_anim_frame]);
				Vertex vrtx_2 = LinearBlending(vrtx_original, current_tpf_gb[next_anim_frame]);
				vrtx.position = vrtx_1.position * (1-t) + vrtx_2.position * t;
				vrtx.normal   = vrtx_1.normal * (1-t) + vrtx_2.normal * t;
			} else {
				vrtx = LinearBlending(vrtx_original, current_tpf_gb[curr_anim_frame]);
			}

			world_positions_array[(i*3)+0] = vrtx.position.x;
			world_positions_array[(i*3)+1] = vrtx.position.y;
			world_positions_array[(i*3)+2] = vrtx.position.z;

			world_normals_array[(i*3)+0] = vrtx.normal.x;
			world_normals_array[(i*3)+1] = vrtx.normal.y;
			world_normals_array[(i*3)+2] = vrtx.normal.z;
		}

		for (int i = 0; i < character->NumTriangles() * 3; i++) {
			triangle_array[i] = character->GetIndex(i);
		}

		glEnable(GL_DEPTH_TEST);
		glEnable(GL_LIGHTING);

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);

		glVertexPointer(3, GL_FLOAT, 0, world_positions_array);
		glNormalPointer(   GL_FLOAT, 0, world_normals_array);

		glDrawElements(GL_TRIANGLES, character->NumTriangles() * 3, GL_UNSIGNED_INT, triangle_array);

		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);

		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
    }


    delete[] world_positions_array;
    delete[] world_normals_array;
    delete[] triangle_array;
}

void Draw() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (float)WIDTH / (float)HEIGHT, 1.0, 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(camera->GetPosition().x, camera->GetPosition().y, camera->GetPosition().z,
              camera->GetTarget().x, camera->GetTarget().y, camera->GetTarget().z,
              0.0, 1.0, 0.0);

    DrawModel();
	DrawTextHint();

    glutSwapBuffers();
}

void MouseEvent(int button, int state, int x, int y) {

    switch(button) {
    case GLUT_LEFT_BUTTON :
        if (state == GLUT_UP) {
            mouse_left_pressed = 0;
        }
        if (state == GLUT_DOWN) {
            mouse_left_pressed = 1;
            last_x = x;
            last_y = y;
        }
        break;

    case GLUT_RIGHT_BUTTON :
        if (state == GLUT_UP) {
            mouse_right_pressed = 0;
        }
        if (state == GLUT_DOWN) {
            mouse_right_pressed = 1;
            last_x = x;
            last_y = y;
        }
        break;

    case GLUT_WHEEL_UP:
        camera->SetPosition( camera->GetPosition() + camera->GetDirection() );
        break;
    case GLUT_WHEEL_DOWN:
        camera->SetPosition( camera->GetPosition() - camera->GetDirection() );
        break;
    }

}

void MouseMoveEvent(int x, int y) {

    if (mouse_left_pressed) {

        int diff_x = x - last_x;
        int diff_y = y - last_y;

        Vector3 offset = camera->GetTarget();

        camera->SetTarget(camera->GetTarget() - offset);
        camera->SetPosition(camera->GetPosition() - offset);

        camera->SetPosition( Matrix_4x4::RotationY(0.01 * -diff_x) * camera->GetPosition() );
        Vector3 axis = Vector3::Normalize(Vector3::Cross( Vector3(0,1,0) , camera->GetDirection() ));
        camera->SetPosition( Matrix_4x4::RotationAngleAxis(axis, 0.01 * diff_y) * camera->GetPosition() );

        camera->SetTarget(camera->GetTarget() + offset);
        camera->SetPosition(camera->GetPosition() + offset);

        last_x = x;
        last_y = y;

    }

}

void KeyEvent(unsigned char key, int x, int y) {
    switch (key) {
		case GLUT_KEY_ESCAPE:
			exit(EXIT_SUCCESS);
			break;

	    case 's':
	    case 'S':
	    	//s - for skeleton
	    	show_skeleton = !show_skeleton;
	    	hint = (show_skeleton) ? "Show Skeleton: ON" : "Show Skeleton: OFF";
	    	break;

	    case 'm':
	    case 'M':
	    	//m - for mesh
	    	show_mesh = !show_mesh;
	    	hint = (show_mesh) ? "Show Mesh: ON" : "Show Mesh: OFF";
	    	break;
	    //controlling animation speed
	    case 'j':
	    case 'J':
	    	frames_per_second -= 1;
	    	frames_per_second = Clamp(frames_per_second, min_frames_per_second, max_frames_per_second);
	    	hint = "Animation Speed: " + Int2String(((int)frames_per_second));
	    	break;
	    case 'k':
	    case 'K':
	    	frames_per_second += 1;
	    	frames_per_second = Clamp(frames_per_second, min_frames_per_second, max_frames_per_second);
	    	hint = "Animation Speed: " + Int2String(((int)frames_per_second));
	    	break;

	    case 'w':
	    case 'W':
	    	current_animation = walk_animation;
	    	current_tpf_gb = walk_tpf_gb;
	    	hint = "Current Animation: Walk";
	    	break;
	    case 'r':
	    case 'R':
	    	current_animation = run_animation;
	    	current_tpf_gb = run_tpf_gb;
	    	hint = "Current Animation: Run";
	    	break;

	    case 'i':
	    case 'I':
	    	interpolate = !interpolate;
	    	hint = (interpolate) ? "Interpolation: ON" : "Interpolation: OFF";
	    	break;
    }
}

enum {
    SMD_STATE_EMPTY = 0,
    SMD_STATE_MESH  = 1,
    SMD_STATE_NODES = 2,
    SMD_STATE_SKEL  = 3
};

void LoadSMDAnimation(std::string filename, Animation** animation) {

    int state = SMD_STATE_EMPTY;

    Skeleton* base = new Skeleton();
    Animation* anim = new Animation();

    std::vector<Joint> joints = std::vector<Joint>();

    std::ifstream f(filename.c_str());

    if (f == NULL) {
        printf("Failed to read file %s\n", filename.c_str());
        fflush(stdout);
        exit(EXIT_FAILURE);
    }

    char line[1024];

    while (!f.eof()) {

        f.getline(line, sizeof(line));

        if (strstr(line, "end"))   {
            state = SMD_STATE_EMPTY;
            continue;
        }
        if (strstr(line, "nodes")) {
            state = SMD_STATE_NODES;
            continue;
        }

        if (strstr(line, "skeleton")) {
            state = SMD_STATE_SKEL;
            base->m_num_joints = joints.size();
            base->m_joints = new Joint[base->m_num_joints];

            for (int i = 0; i < base->m_num_joints; i++) {
                base->m_joints[i] = joints[i];
            }

            continue;
        }

        if (strstr(line, "time")) {
            anim->AddFrame(base);
        }

        if (state == SMD_STATE_NODES) {
            char name[256];
            int id, parent;
            if (sscanf(line, "%i \"%[^\"]\" %i", &id, name, &parent) == 3) {
                joints.push_back(Joint(id, parent));
            }
        }

        if (state == SMD_STATE_SKEL) {
            int id;
            float x, y, z, rx, ry, rz;
            if (sscanf(line, "%i %f %f %f %f %f %f", &id, &x, &y, &z, &rx, &ry, &rz) == 7) {

                Skeleton* frame = anim->GetFrame(anim->NumFrames()-1);

                /* Swap y and z */
                frame->m_joints[id].position = Vector3(x, z, y);

                Matrix_4x4 rotation = Matrix_4x4::RotationEuler(rx, ry, rz);
                Matrix_4x4 handedflip = Matrix_4x4(1,0,0,0,  0,0,1,0,  0,1,0,0,  0,0,0,1);

                rotation = handedflip * rotation;
                rotation = rotation * handedflip;

                frame->m_joints[id].rotation = Matrix_4x4::Transpose(rotation);
            }
        }

    }

    delete base;

    (*animation) = anim;

}

void LoadSMDCharacter(std::string filename, Mesh** character) {

    int state = SMD_STATE_EMPTY;

    std::vector<Vertex> verts = std::vector<Vertex>();
    std::vector<int> tris = std::vector<int>();

    std::ifstream f(filename.c_str());

    if (f == NULL) {
        printf("Failed to read file %s\n", filename.c_str());
        fflush(stdout);
        exit(EXIT_FAILURE);
    }

    char line[1024];

    while (!f.eof()) {

        f.getline(line, sizeof(line));

        if (strstr(line, "end")) {
            state = SMD_STATE_EMPTY;
            continue;
        }
        if (strstr(line, "triangles")) {
            state = SMD_STATE_MESH;
            continue;
        }

        if (state == SMD_STATE_MESH) {

            int id = 0, l1_id = 0, l2_id = 0, l3_id = 0;
            int num_links = 0;
            float x, y, z, nx, ny, nz, u, v, l1_amount = 0, l2_amount = 0, l3_amount = 0;

            if (sscanf(line, "%i %f %f %f %f %f %f %f %f %i %i %f %i %f %i %f",
                       &id, &x, &y, &z, &nx, &ny, &nz, &u, &v, &num_links,
                       &l1_id, &l1_amount, &l2_id, &l2_amount, &l3_id, &l3_amount) > 10) {

                /* Swap y and z axis */
                Vertex vert;
                vert.position = Vector3(x, z, y);
                vert.normal = Vector3(nx, nz, ny);
                vert.weight_ids = Vector3(l1_id, l2_id, l3_id);
                vert.weight_amounts = Vector3(l1_amount, l2_amount, l3_amount);

                verts.push_back(vert);
                tris.push_back(verts.size()-1);
            }
        }

    }

    f.close();

    Mesh* mesh = new Mesh();
    mesh->m_num_vertices = verts.size();
    mesh->m_num_triangles = tris.size() / 3;
    mesh->m_vertices = new Vertex[mesh->m_num_vertices];
    mesh->m_triangles = new int[mesh->m_num_triangles * 3];

    for(int i = 0; i < mesh->m_num_vertices; i++) {
        mesh->m_vertices[i] = verts[i];
    }

    for(int i = 0; i < mesh->m_num_triangles; i++) {
        mesh->m_triangles[i*3+0] = tris[i*3+2];
        mesh->m_triangles[i*3+1] = tris[i*3+1];
        mesh->m_triangles[i*3+2] = tris[i*3+0];
    }

    (*character) = mesh;

}

int main(int argc, char **argv) {


    camera = new Camera(Vector3(20, 30, 50), Vector3(0, 15, 0));

    LoadSMDCharacter("./resources/character.smd", &character);
    LoadSMDAnimation("./resources/rest_animation.smd", &rest_animation);
    LoadSMDAnimation("./resources/run_animation.smd",  &run_animation);
    LoadSMDAnimation("./resources/walk_animation.smd",  &walk_animation);

    printf("rest_animation -> number of frames: %d \n", rest_animation->NumFrames());
    printf("run_animation -> number of frames: %d \n", run_animation->NumFrames());
    printf("walk_animation -> number of frames: %d \n", walk_animation->NumFrames());

    //compute initial transforms
    ComputeRestTransLC(rest_trans_lc, rest_animation->GetFrame(0));
    ComputeTransPerFrameGB(rest_tpf_gb, rest_animation);
    ComputeTransPerFrameGB(run_tpf_gb, run_animation);
    ComputeTransPerFrameGB(walk_tpf_gb, walk_animation);

    //don't need to free current_animation
    printf("current animation is walking");
    //current_animation = rest_animation;
    //current_tpf_gb = rest_tpf_gb;
    //current_animation = walk_animation;
    //current_tpf_gb = walk_tpf_gb;
    current_animation = run_animation;
    current_tpf_gb = run_tpf_gb;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE|GLUT_DEPTH|GLUT_MULTISAMPLE);

    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Skinning");

    glClearColor(0.5, 0.5, 0.5, 1.0);

    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);

    GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat mat_shininess[] = { 100.0 };
    GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };

    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    glEnable(GL_LIGHT0);

    glutDisplayFunc(Draw);
    glutIdleFunc(Update);
    glutMouseFunc(MouseEvent);
    glutMotionFunc(MouseMoveEvent);
    glutKeyboardFunc(KeyEvent);

    glutMainLoop();


    delete camera;
    delete character;
    delete rest_animation;
    delete run_animation;
    //there was a bug in the original code. The memory for walk animation hasn't been freed
    delete walk_animation;
}


