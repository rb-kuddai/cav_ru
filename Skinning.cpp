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

static Mesh* character = NULL;

static Animation* rest_animation = NULL;
static Animation* run_animation = NULL;
static Animation* walk_animation = NULL;

/* This timer can be used to cycle through the frames of animation */
static float timer = 0;

void Update() {
    timer += 0.05;
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

static void DrawModel() {

    float* world_positions_array = new float[character->NumVertices() * 3];
    float* world_normals_array = new float[character->NumVertices() * 3];
    int* triangle_array = new int[character->NumTriangles() * 3];

    /*
    ** TODO: Uncomment this once `JointTransform` is implemented to draw
    ** kotik!!!! YEAH
    **       the skeleton of the character in the rest pose.
    */

    // Skeleton* rest = rest_animation->GetFrame(0);
    // DrawSkeleton(rest);

    for (int i = 0; i < character->NumVertices(); i++) {

        Vector3 position = character->GetVertex(i).position;
        Vector3 normal = character->GetVertex(i).normal;

        /*
        ** TODO: Transform position and normal using rest pose and some
        **       other animated pose from one of the animations
        */

        world_positions_array[(i*3)+0] = position.x;
        world_positions_array[(i*3)+1] = position.y;
        world_positions_array[(i*3)+2] = position.z;

        world_normals_array[(i*3)+0] = normal.x;
        world_normals_array[(i*3)+1] = normal.y;
        world_normals_array[(i*3)+2] = normal.z;
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

    delete[] world_positions_array;
    delete[] world_normals_array;
    delete[] triangle_array;

}

static const int WIDTH = 800;
static const int HEIGHT = 600;

static int mouse_left_pressed = 0;
static int mouse_right_pressed = 0;

static int last_x = -1;
static int last_y = -1;

static Camera* camera = NULL;

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

}

//kotik

