import numpy as np
import random
import pybullet as p
from  additional import gazebo_world_parser

import os
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

X = 0
Y = 1
Z = 2

class WorldCreator():
    def __init__(self,
                pybullet_client : p,
                spinning_friction = 0.0063,
                lateral_friction = 1.0
                ) -> None:
        self._pybullet_client = pybullet_client
        self.spinning_friction = spinning_friction
        self.lateral_friction = lateral_friction

        self.models_addr = parentdir + "/models/"
        self.gazebo_world_addr = parentdir + "/worlds/"
        # print("ADDRESS")
        # print(self.models_addr)

    def create_world(self, world_name : str, lateralFriction=1.0, spinningFriction=0.0063):
        self.spinning_friction = spinningFriction
        self.lateral_friction = lateralFriction

        if world_name.find("gazebo") == -1:
            if world_name == "empty":
                self._create_empty_world()
            elif world_name == "random1":
                self._create_random1_world()
            elif world_name == "random2":
                self._create_random2_world()
            elif world_name == "random3":
                self._create_random3_world()
            elif world_name == "random_blocks":
                self._create_random_blocks()
            elif world_name == "boxes":
                self._create_boxes()
            elif world_name == "stairs":
                self._create_stairs()
            else:
                self._create_empty_world()
        else:
            
            self._create_gazebo_world(world_name.replace('gazebo_', ''))

    def _create_empty_world(self):
        planeShape = self._pybullet_client.createCollisionShape(shapeType=self._pybullet_client.GEOM_PLANE)
        ground_id = self._pybullet_client.createMultiBody(0, planeShape)
        self._pybullet_client.resetBasePositionAndOrientation(ground_id, [0, 0, 0.0], [0, 0, 0, 1])
        self._pybullet_client.changeDynamics(ground_id, -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)

    def _create_random1_world(self, heightPerturbationRange=0.07, numHeightfieldRows=256, numHeightfieldColumns=256):
        heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns
        for j in range(int(numHeightfieldColumns/2)):
            for i in range(int(numHeightfieldRows/2)):
                height = random.uniform(0, heightPerturbationRange)
                heightfieldData[2*i+2*j*numHeightfieldRows] = height
                heightfieldData[2*i+1+2*j*numHeightfieldRows] = height
                heightfieldData[2*i+(2*j+1)*numHeightfieldRows] = height
                heightfieldData[2*i+1+(2*j+1)*numHeightfieldRows] = height
        terrainShape = self._pybullet_client.createCollisionShape(
            shapeType=self._pybullet_client.GEOM_HEIGHTFIELD,
            meshScale=[.1, .1, 1],
            heightfieldTextureScaling=(numHeightfieldRows-1)/2,
            heightfieldData=heightfieldData,
            numHeightfieldRows=numHeightfieldRows,
            numHeightfieldColumns=numHeightfieldColumns)
        ground_id = self._pybullet_client.createMultiBody(0, terrainShape)
        self._pybullet_client.changeVisualShape(ground_id, -1, rgbaColor=[0, 1, 0, 0.7])
        self._pybullet_client.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
        self._pybullet_client.changeDynamics(ground_id, -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)

    def _create_random2_world(self):
        terrain_shape = self._pybullet_client.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                meshScale=[.5, .5, .5],
                fileName="heightmaps/ground0.txt",
                heightfieldTextureScaling=128)
        ground_id = self._pybullet_client.createMultiBody(0, terrain_shape)
        print(self.models_addr)
        textureId = self._pybullet_client.loadTexture(self.models_addr + "mors_sim/grass.png")
        self._pybullet_client.changeVisualShape(ground_id, -1, textureUniqueId=textureId)
        self._pybullet_client.resetBasePositionAndOrientation(ground_id, [0, 0, 0.25], [0, 0, 0, 1])
        self._pybullet_client.changeDynamics(ground_id, -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)

    def _create_random3_world(self):
        terrain_shape = self._pybullet_client.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                meshScale=[.01, .01, .45],
                fileName=self.models_addr + "mors_sim/wm_height_out.png",
                heightfieldTextureScaling=128)
        terrain  = self._pybullet_client.createMultiBody(0, terrain_shape)
        self._pybullet_client.resetBasePositionAndOrientation(terrain, [0, 0, -0.2], [0, 0, 0, 1])
        self._pybullet_client.changeDynamics(terrain, -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)

        # num_boxes = 50

        # # self._create_empty_world()

        # boxShape = [0]*num_boxes
        # ground_id = [0]*num_boxes

        # z_pos = 0
        # for i in range(1, num_boxes):
        #     size_x = random.uniform(0.1, sizePerturbationRange[X])
        #     size_y = random.uniform(0.1, sizePerturbationRange[Y])
        #     x_pos = random.uniform(0.1, sizePerturbationRange[X])
        #     z_pos += random.uniform(-z_pos_range, z_pos_range)

        #     boxShape[i] = self._pybullet_client.createCollisionShape(shapeType=self._pybullet_client.GEOM_BOX, halfExtents=[size_x/2, size_y/2, 1.0])
        #     ground_id[i] = self._pybullet_client.createMultiBody(0, boxShape[i])
        #     self._pybullet_client.resetBasePositionAndOrientation(ground_id[i], [i/2, 0, z_pos], [0,0,0,1])
        #     self._pybullet_client.changeDynamics(ground_id[i], -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)
        

    def _create_random_blocks(self, sizePerturbationRange=[0.6, 0.1, 0.07], rpyPerturbationRange=[0, 0, 1.56]):
        num_boxes = 20
        x_initial_pos = 0.3
        z_initial_pos = 0.1

        planeShape = self._pybullet_client.createCollisionShape(shapeType=self._pybullet_client.GEOM_PLANE)
        ground_id = self._pybullet_client.createMultiBody(0, planeShape)
        self._pybullet_client.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
        self._pybullet_client.changeDynamics(ground_id, -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)

        boxShape = [0]*num_boxes
        ground_id = [0]*num_boxes

        x_pos = x_initial_pos
        for i in range(num_boxes):
            size_x = random.uniform(0.01, sizePerturbationRange[X])
            size_y = random.uniform(0.01, sizePerturbationRange[Y])
            size_z = random.uniform(0.002, sizePerturbationRange[Z])

            yaw_pos = random.uniform(0.002, rpyPerturbationRange[Z])
            x_pos += random.uniform(sizePerturbationRange[X]*0.15, sizePerturbationRange[X]*0.25)
            y_pos = random.uniform(-sizePerturbationRange[X]*1.0, sizePerturbationRange[Y]*1.0)
            z_pos = z_initial_pos + size_z*3

            orient = p.getQuaternionFromEuler([0,0,yaw_pos])

            boxShape[i] = self._pybullet_client.createCollisionShape(shapeType=self._pybullet_client.GEOM_BOX, halfExtents=[size_x/2, size_y/2, size_z/2])
            ground_id[i] = self._pybullet_client.createMultiBody(0, boxShape[i])
            self._pybullet_client.resetBasePositionAndOrientation(ground_id[i], [x_pos, y_pos, z_pos], orient)
            self._pybullet_client.changeDynamics(ground_id[i], -1, mass=1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)

    def _create_boxes(self):
        num_boxes = 10
        self._create_empty_world()

        boxShape = [0]*num_boxes
        ground_id = [0]*num_boxes
        for i in range(1, num_boxes):
            boxShape[i] = self._pybullet_client.createCollisionShape(shapeType=self._pybullet_client.GEOM_BOX, halfExtents=[0.1/2, 1/2, i/100])
            ground_id[i] = self._pybullet_client.createMultiBody(0, boxShape[i])
            self._pybullet_client.resetBasePositionAndOrientation(ground_id[i], [i/2, 0, 0], [0,0,0,1])
            self._pybullet_client.changeDynamics(ground_id[i], -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)

    def _create_stairs(self):
        num_boxes = 10
        step_height = 0.15
        step_length = 0.35
        distance = 0.3
        self._create_empty_world()

        boxShape = [0]*num_boxes
        ground_id = [0]*num_boxes
        x_pos = distance
        for i in range(1, num_boxes):
            x_pos += step_length
            boxShape[i] = self._pybullet_client.createCollisionShape(shapeType=self._pybullet_client.GEOM_BOX, halfExtents=[step_length/2, 1/2, step_height*i])
            ground_id[i] = self._pybullet_client.createMultiBody(0, boxShape[i])
            self._pybullet_client.resetBasePositionAndOrientation(ground_id[i], [x_pos, 0, 0], [0,0,0,1])
            self._pybullet_client.changeDynamics(ground_id[i], -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)

        for i in range(1, num_boxes):
            x_pos += step_length
            boxShape[i] = self._pybullet_client.createCollisionShape(shapeType=self._pybullet_client.GEOM_BOX, halfExtents=[step_length/2, 1/2, step_height*(num_boxes-i)])
            ground_id[i] = self._pybullet_client.createMultiBody(0, boxShape[i])
            self._pybullet_client.resetBasePositionAndOrientation(ground_id[i], [x_pos, 0, 0], [0,0,0,1])
            self._pybullet_client.changeDynamics(ground_id[i], -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)


    def _create_gazebo_world(self, world_name : str):
        os.chdir(parentdir)
        self._pybullet_client.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        gazebo_world_parser.parseWorld(self._pybullet_client, filepath=self.gazebo_world_addr + world_name + ".world")
        self._pybullet_client.configureDebugVisualizer(shadowMapResolution=8192)
        self._pybullet_client.configureDebugVisualizer(shadowMapWorldSize=25)
        self._pybullet_client.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    