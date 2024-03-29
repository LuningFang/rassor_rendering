import bpy
import math
import random
import os
import sys
import mathutils



# define different camera settings, this will be used to create different camera location, pointed at different location, and write different image name 
# there will be two types of camera settings, one is for the side view of the vehicle, the other is for the focused view close to the wheel and drum 
# the side view is for the whole vehicle, the focused view is for the drum and the wheel
camera_settings = {
    "side_view": {
        "location": (0.25, -3, 0.25),
        "point_at": (0, 0, 0),
        "file_name": "side_view"
    },
    "focused_view": {
        "location": (-0.1, -0.1,  0.5 ),
        "point_at": ( 0.5,  0.1, -0.25),
        "file_name": "focused_view"
    }
}


#============ find the position that the camera points at
def point_at(obj, target, roll=0):
    """
    Rotate obj to look at target
    :arg obj: the object to be rotated. Usually the camera
    :arg target: the location (3-tuple or Vector) to be looked at
    :arg roll: The angle of rotation about the axis from obj to target in radians. 
    """
    if not isinstance(target, mathutils.Vector):
        target = mathutils.Vector(target)
    loc = obj.location
    # direction points from the object to the target
    direction = target - loc

    quat = direction.to_track_quat('-Z', 'Y')
    
    # /usr/share/blender/scripts/addons/add_advanced_objects_menu/arrange_on_curve.py
    quat = quat.to_matrix().to_4x4()
    rollMatrix = mathutils.Matrix.Rotation(roll, 4, 'Z')

    # remember the current location, since assigning to obj.matrix_world changes it
    loc = loc.to_tuple()
    #obj.matrix_world = quat * rollMatrix
    # in blender 2.8 and above @ is used to multiply matrices
    # using * still works but results in unexpected behaviour!
    obj.matrix_world = quat @ rollMatrix
    obj.location = loc

#============ find the center right position of the chassis
def get_chassis_right_center(positions_drum):
    # given position drum, find the bounding box of the drum
    x_min = 1000
    x_max = -1000
    z_min = 1000
    z_max = -1000
    y_min = 1000
    y_max = -1000

    for pos in positions_drum:
        x, y, z = pos
        if x < x_min:
            x_min = x
        if x > x_max:
            x_max = x
        if z < z_min:
            z_min = z
        if z > z_max:
            z_max = z
        if y < y_min:
            y_min = y
        if y > y_max:
            y_max = y

    # get the center of the drum, this should be the position camera is pointing at
    vehicle_center_x = (x_min + x_max) / 2
    vehicle_center_z = (z_min + z_max) / 2
    vehicle_y_min = y_min

    return vehicle_center_x, vehicle_y_min, vehicle_center_z


# read particle files and create both soil and drum particles
def read_particle_file_half_drum(particle_dir):
    in_particle_file = particle_dir + "BCE_Rigid" + str(k) + ".csv"
    print("read drum file: " + in_particle_file)
    line_count = 0
    count = 0

    positions_drum = []
    positions_soil = []
    # in_particle_file has x, y, z compoenents, find the minimum of y
    y_min = 1000

    with open(in_particle_file) as f:
        for line in f:
            #if count > 9000000: break
            if line_count == 0:
                line_count += 1
                continue
            else:
                line_count += 1
                # you have to parse "x", "y", "z" and "r" from the variable "line"
                line_seg = line.split(",")
                #print(line_seg)
                y = float(line_seg[1])

                if y < y_min:
                    y_min = y

        print("y_min: " + str(y_min))
        f.seek(0)
        line_count = 0
        count = 0
        for line in f:
            #if count > 9000000: break
            if line_count == 0:
                line_count += 1
                continue
            else:
                line_count += 1
                # you have to parse "x", "y", "z" and "r" from the variable "line"
                line_seg = line.split(",")
                #print(line_seg)
                x, y, z = float(line_seg[0]), float(line_seg[1]), float(line_seg[2])

                if y > y_min + 0.02:
                    positions_drum.append((x,y,z))
                    count = count + 1
    print("total number of drum particles " + str(count))

    # read fluid particles, ignore everything with y coordinates larger than 0.045 
    in_particle_file = particle_dir + "fluid" + str(k) + ".csv"
    print("read particle file: " + in_particle_file)
    line_count = 0
    count = 0
    with open(in_particle_file) as f:
        for line in f:
            #if count > 9000000: break
            if line_count == 0:
                line_count += 1
                continue
            else:
                line_count += 1
                # you have to parse "x", "y", "z" and "r" from the variable "line"
                line_seg = line.split(",")
                #print(line_seg)
                x, y, z =  float(line_seg[0]), float(line_seg[1]), float(line_seg[2])

                # do not count fluid particles outside the drum
                if abs(y) > 0.65:
                    continue
                
                positions_soil.append((x,y,z))
                count = count + 1
    print("total number of fluid particles " + str(count))


    return positions_drum, positions_soil

#radius_particle = 0.5
start_frame = 0
# end_frame = 10
end_frame = 1

step_format = "%04d"
ground_plane_pos = 0


# list of obj names
obj_names = ["chassis", "arm_B", "arm_F", "Wheel_LF", "Wheel_RF", "Wheel_LB", "Wheel_RB", "razor_F", "razor_B"]

my_start_frame = 0
my_end_frame = 100

particle_radius = 0.002

# sim_folder="/srv/home/fang/RASSOR/my-fork/build/bin/DEMO_OUTPUT/" + sys.argv[4] + "/"
sim_folder = "C:/Users/fang/Documents/Rassor/build/bin/Release/DEMO_OUTPUT/" + sys.argv[4] + "/" 
particle_dir = sim_folder + "particles/"
mesh_dir     = sim_folder + "rover/"
out_dir   = sim_folder + "images/"

os.makedirs(out_dir, exist_ok=True)

# command line input for camera view type
CAMERA_VIEW = sys.argv[5]
print("camera view: " + CAMERA_VIEW)

# for k in a range start and end with interval of 10 

for k in range(my_start_frame, my_end_frame, 1):
    try:
        
        positions_drum, positions_soil = read_particle_file_half_drum(particle_dir)


        vehicle_center_x, vehicle_y_min, vehicle_center_z = get_chassis_right_center(positions_drum)

        bpy.ops.wm.read_factory_settings(use_empty=True)

        scene = bpy.context.scene
        scene.objects.keys()

        bpy.ops.mesh.primitive_plane_add(size=20.0, calc_uvs=True, enter_editmode=False, align='WORLD',
                                         location=(0.0, 0.0, ground_plane_pos))

        ov=bpy.context.copy()
        ov['area']=[a for a in bpy.context.screen.areas if a.type=="VIEW_3D"][0]
        # bpy.ops.transform.rotate(ov,value=-(math.pi * 0.5), orient_axis='X')  # value = Angle


        """ -------------------------PARTICLE SYSTEM TEST------------------------------------------------------ """
        context = bpy.context
        # Create the blue material
        material_drum = bpy.data.materials.new(name="BlueMaterial")
        # material_drum.diffuse_color = (0.2, 0.2, 0.6, 1)  # purple color
        # material_drum.diffuse_color = (0.1, 0.1, 0.1, 0.1)  # dark grey
        material_drum.diffuse_color = (0.3, 0, 0, 1)  # dark red

        # Create the gray material
        material_soil = bpy.data.materials.new(name="GrayMaterial")
        # material_soil.diffuse_color = (0.4, 0.4, 0.4, 0.1)  # light Gray color
        material_soil.diffuse_color = (0.1, 0.1, 0.1, 0.1)  # Gray color

        # Create the blue icosphere and set the blue material
        bpy.ops.mesh.primitive_ico_sphere_add(radius=1, location=(50, 50, 50))
        ico_blue = context.object
        ico_blue.data.materials.append(material_drum)

        # instance object
        bpy.ops.mesh.primitive_ico_sphere_add(radius=1, location=(50, 50, 50))
        ico_gray = context.object
        ico_gray.data.materials.append(material_soil)

        # cube with ps
        bpy.ops.mesh.primitive_cube_add(size=0.0001)
        blue_cube = context.object

        # Create the particle system for blue particles
        blue_ps = blue_cube.modifiers.new("drum_particles", 'PARTICLE_SYSTEM').particle_system
        blue_psname = blue_ps.name

        # Configure particle system settings for blue particles
        blue_ps.settings.count = len(positions_drum)
        blue_ps.settings.lifetime = 1000
        blue_ps.settings.frame_start = blue_ps.settings.frame_end = 1
        blue_ps.settings.render_type = "OBJECT"
        blue_ps.settings.instance_object = ico_blue

        
        bpy.ops.mesh.primitive_cube_add(size=0.0001)
        gray_cube = context.object

        
        # ps
        gray_ps = gray_cube.modifiers.new("SomeName", 'PARTICLE_SYSTEM').particle_system
        gray_psname = gray_ps.name


        gray_ps.settings.count = len(positions_soil)
        gray_ps.settings.lifetime = 1000
        gray_ps.settings.frame_start = gray_ps.settings.frame_end = 1
        gray_ps.settings.render_type = "OBJECT"
        gray_ps.settings.instance_object = ico_gray

        def particle_handler_soil(scene, depsgraph):
            ob = depsgraph.objects.get(gray_cube.name)
            if ob:
                ps = ob.particle_systems[gray_psname]
                f = scene.frame_current
                for m, particle in enumerate(ps.particles):
                    setattr(particle, "location", positions_soil[m])
                    setattr(particle, "size", particle_radius)

        def particle_handler_drum(scene, depsgraph):
            ob = depsgraph.objects.get(blue_cube.name)
            if ob:
                ps = ob.particle_systems[blue_psname]
                f = scene.frame_current
                for m, particle in enumerate(ps.particles):
                    setattr(particle, "location", positions_drum[m])
                    setattr(particle, "size", particle_radius)

        # Clear the post frame handler
        bpy.app.handlers.frame_change_post.clear()

        # Register our particleSetter with the post frame handler
        bpy.app.handlers.frame_change_post.append(particle_handler_soil)
        # bpy.app.handlers.frame_change_post.append(particle_handler_drum)

        # Trigger frame update
        bpy.context.scene.frame_current = 2

        """"Add obj mesh to the scene"""

        # # Read the OBJ file and import the mesh
        for obj_name in obj_names:

            in_mesh_file = mesh_dir + obj_name + "_" + str(k) + ".obj"
            print("Reading OBJ file: " + in_mesh_file)
            imported_object_0 = bpy.ops.import_scene.obj(filepath=in_mesh_file)

            # Access the imported mesh object
            mesh_object = bpy.context.selected_objects[0]

            # Set the material to be transparent
            if mesh_object.material_slots:
                material = mesh_object.material_slots[0].material
                material.use_nodes = True
                material.node_tree.nodes["Principled BSDF"].inputs["Alpha"].default_value = 1.0  # Adjust alpha value as needed

                if obj_name.startswith("razor"):
                    material.node_tree.nodes["Principled BSDF"].inputs["Alpha"].default_value = 0.2
                    material.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.2, 0.2, 0.6, 1)
                
                if obj_name.startswith("chassis") or obj_name.startswith("arm") or obj_name.startswith("Wheel"):
                    material.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.2, 0.2, 0.6, 1)

            ov = bpy.context.copy()
            ov['area'] = [a for a in bpy.context.screen.areas if a.type == "VIEW_3D"][0]
            bpy.ops.transform.rotate(ov, value=(math.pi * 0.5), orient_axis='X')


        """ -----------PARTICLE SYSTEM TEST END-------------------------------------------- """

        bpy.context.view_layer.update()

        bpy.ops.object.camera_add(enter_editmode=False, align='WORLD', rotation=(1.4, 0.0, 0.0), scale=(5.0, 5.0, 5.0))

        # Set up rotational camera
        cam = bpy.data.objects["Camera"]
        # side view of the vehicle
        # cam.location = (vehicle_center_x + 0.25, -3, vehicle_center_z + 0.25)
        # point_at(cam,  (vehicle_center_x, vehicle_y_min, vehicle_center_z), roll=math.radians(0))

        cam_relative_loc = camera_settings[CAMERA_VIEW]["location"]
        cam_point_at_loc = camera_settings[CAMERA_VIEW]["point_at"]
        cam.location = (vehicle_center_x + cam_relative_loc[0], vehicle_y_min + cam_relative_loc[1], vehicle_center_z + cam_relative_loc[2])
        point_at(cam,  (vehicle_center_x + cam_point_at_loc[0], vehicle_y_min + cam_point_at_loc[1], vehicle_center_z + cam_point_at_loc[2]), roll=math.radians(0))
        
        
        # (vehicle_center_x - 0.1, vehicle_y_min - 0.1, vehicle_center_z + 0.5)
        # point_at(cam,  (vehicle_center_x + 0.5, vehicle_y_min + 0.1, vehicle_center_z - 0.25), roll=math.radians(0))


        # bpy.ops.object.camera_add(enter_editmode=False, align='WORLD', location=(drum_center_x, -3.*unit_conv, 1.*unit_conv),
        #                       rotation=(math.pi/2., 0.0, 0.0), scale=(5.0, 5.0, 5.0))


        scene.camera = bpy.context.object
        scene.cycles.device = 'GPU'

        prefs = bpy.context.preferences
        cprefs = prefs.addons['cycles'].preferences

        # Attempt to set GPU device types if available
        for compute_device_type in ('CUDA', 'OPENCL', 'NONE'):
            try:
                cprefs.compute_device_type = compute_device_type
                break
            except TypeError:
                pass

        # Enable all CPU and GPU devices
        cprefs.get_devices()
        for device in cprefs.devices:
            device.use = True

        # create light datablock, set attributes
        light_data = bpy.data.lights.new(name="light_2.80", type='POINT')
        light_data.energy = 5000

        # create new object with our light datablock
        light_object = bpy.data.objects.new(name="light_2.80", object_data=light_data)

        # link light object
        bpy.context.collection.objects.link(light_object)

        # clip_end setting, make it large enough
        bpy.context.scene.camera.data.clip_end = 10

        # make it active
        bpy.context.view_layer.objects.active = light_object

        # change location
        light_object.location = (vehicle_center_x, -5., 10.)

        # Create a second light data block
        second_light_data = bpy.data.lights.new(name="second_light", type='POINT')
        second_light_data.energy = 5000  # Adjust energy as needed

        # Create a new object with the second light data block
        second_light_object = bpy.data.objects.new(name="second_light", object_data=second_light_data)

        # Link the second light object to the scene
        bpy.context.collection.objects.link(second_light_object)

        # Change the location of the second light to balance the shadows
        second_light_object.location = (vehicle_center_x, 5., 10.)  # Adjust location as needed




        bpy.context.scene.render.engine = 'CYCLES'
        bpy.context.scene.cycles.device = 'GPU'
        #bpy.context.scene.render.resolution_percentage = 200
        # bpy.context.scene.cycles.samples = 512
        bpy.context.scene.cycles.samples = 200

        bpy.context.scene.render.resolution_x = 1080
        bpy.context.scene.render.resolution_y = 960

        bpy.context.scene.render.filepath = out_dir + camera_settings[CAMERA_VIEW]["file_name"] + "_" +step_format%k + ".png"
        # bpy.context.scene.render.image_settings.compression = 50
        bpy.context.scene.render.image_settings.color_mode = 'RGBA'
        bpy.context.scene.render.image_settings.file_format = 'PNG'
        bpy.ops.render.render(write_still=True)
    except:
        continue


