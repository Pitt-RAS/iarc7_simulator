import bge
from bge import texture
from mathutils import Euler
import math
import control

# all units in mks

QUAD_OBJECT_NAME = 'F450'

NUMBER_OF_PROPS = 4

PROP_DIAMETER = 0.2
PROP_PITCH = 0.1
PROP_MAX_SPEED = 8214 # ideally 14985 # rpm
AIR_DENSITY = 1.225

PROP_AREA = math.pi * (PROP_DIAMETER / 2)**2
VOLUME_PER_REV = PROP_PITCH * PROP_AREA
MASS_PER_REV = AIR_DENSITY * VOLUME_PER_REV

def update():
    control.loop()

    quad = bge.logic.getCurrentController().owner
    throttle = quad['throttle']
    heading = quad['heading'] * math.pi / 180
    pitch = quad['pitch'] * math.pi / 180
    roll = quad['roll'] * math.pi / 180
    orientation = (-heading, pitch, roll)
    quad.worldOrientation = Euler(orientation, 'ZYX')

    velocity = quad.getLinearVelocity(True)[2]

    prop_speed = throttle * PROP_MAX_SPEED * (1/60.0)
    thrust = (prop_speed * PROP_PITCH - velocity) * MASS_PER_REV * prop_speed

    quad.applyForce((0, 0, NUMBER_OF_PROPS * thrust), True)

def set_throttle(throttle):
    if throttle > 1:
        throttle = 1
    elif throttle < 0:
        throttle = 0
    bge.logic.getCurrentScene().objects[QUAD_OBJECT_NAME]['throttle'] = throttle

def set_heading(heading):
    bge.logic.getCurrentScene().objects[QUAD_OBJECT_NAME]['heading'] = heading

def set_pitch(pitch):
    bge.logic.getCurrentScene().objects[QUAD_OBJECT_NAME]['pitch'] = pitch

def set_roll(roll):
    bge.logic.getCurrentScene().objects[QUAD_OBJECT_NAME]['roll'] = roll

def get_throttle():
    return bge.logic.getCurrentScene().objects[QUAD_OBJECT_NAME]['throttle']

def get_heading():
    return bge.logic.getCurrentScene().objects[QUAD_OBJECT_NAME]['heading']

def get_pitch():
    return bge.logic.getCurrentScene().objects[QUAD_OBJECT_NAME]['pitch']

def get_roll():
    return bge.logic.getCurrentScene().objects[QUAD_OBJECT_NAME]['roll']

def get_altitude():
    return bge.logic.getCurrentScene().objects[QUAD_OBJECT_NAME].worldPosition[2]

inited = False
camera_image = None
def get_camera_image():
    if not inited:
        obj = bge.logic.getCurrentScene().objects['Empty']
        for child in obj.children:
            if 'CameraRobot' in child.name:
                camera = child
            if 'CameraMesh' in child.name:
                screen = child
                mesh = child.meshes[0]
                for material in mesh.materials:
                    material_index = material.getMaterialIndex()
                    mesh_material_name = mesh.getMaterialName(material_index)
                    if 'MAScreenMat' in mesh_material_name:
                        material_name = mesh_material_name
        s = [scene for scene in bge.logic.getSceneList() if scene.name == 'S.256x256'][0]
        img_renderer = texture.ImageRender(s, camera)
        mat_id = texture.materialID(screen, material_name)
        global camera_image
        camera_image = texture.Texture(screen, mat_id)
        camera_image.source = img_renderer
        camera.lens = 35.0
        camera.near = 0.1
        camera.far = 100.0
        camera_image.source.background = [143]*4
        camera_image.source.capsize = [512, 512]
    camera_image.source.refresh()
    print(camera_image.source.valid)
    camera_image.refresh(True)
    #if s == None:
    #    global s
    #    #s = bge.logic.addScene('S.256x256', 0)
    #    s = [scene for scene in bge.logic.getSceneList() if scene.name == 'S.256x256'][0]
    #scene = s
    #quad = scene.objects[QUAD_OBJECT_NAME]
    ##camera = quad.children['ForwardCamera']
    #camera = scene.objects['Camera']

    ##matID = texture.materialID(scene.objects['Cube.003'], 'MAMaterial.003')

    ##if t == None:
    ##    global t
    ##    t = texture.Texture(scene.objects['Cube.003'], matID)
    ##    t.source = texture.ImageRender(scene, camera)
    #print(texture.ImageRender(scene, camera).image)

    #print(t.source.valid)
    #print(t.source.image)
    #t.refresh(True)
