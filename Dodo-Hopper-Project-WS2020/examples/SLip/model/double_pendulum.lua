--This is a 2 dof planar pendulum

print("Lua: Constructing 2 dof pendulum")

inertiaMatrix = {
    {0.0833,0.     ,0.    },
    {0.    ,0.00833,0.    },
    {0.    ,0.     ,0.0833}        
}

nullMatrix = {
    {0.    ,0.     ,0.    },
    {0.    ,0.     ,0.    },
    {0.    ,0.     ,0.    }
}


print("Lua: Body mass and geometry properties")
linkProperties= {   mass    = 1., 
                    com     = {0.,-1.,0.}, 
                    inertia = inertiaMatrix}

baseProperties= {   mass    = 1.,
                    com     = {0.,0.,0.},
                    inertia = nullMatrix}
print("Lua: Putting bodies in a table")
bodies = {
    link  = linkProperties,
    floatingBase  = baseProperties}

print("Lua: Making a table of joints")
joints = {
    rotational_z = {
        {0.,0.,1.,0.,0.,0.}   
    },
    floating = {
        { 0., 0., 0., 1., 0., 0.},
        { 0., 0., 0., 0., 1., 0.},
	{ 0., 0., 1., 0., 0., 0.}
    }
}
print("Lua: Making the meshes")
meshes = {
    link = {
        dimensions = { 0.05, 1.0, 0.05 },
        color = { 1, 0, 0 },
        mesh_center = { 0, -0.5, 0 },
        src = "unit_cube.obj",
    },
    floatingBase = {
        dimensions = {0.2,0.2,0.2},
        color       = { 0, 1, 0 },
        mesh_center = { 0, 0.0, 0 },
        src = "unit_sphere_medres.obj",
    }
}

print("Lua: Making the model")
model = {
    gravity = {0., -9.81, 0.},

    frames = {   
             {
            name    = "floatingBase",
            parent  = "ROOT",
            visuals = {meshes.floatingBase},
            body    = bodies.floatingBase,
            joint   = joints.floating,
            joint_frame = {
                r = {0, 0, 0},
            }
        },   
        {
            name    = "link1",
            parent  = "floatingBase",
            visuals = {meshes.link},
            body    = bodies.link,
            joint   = joints.rotational_z,
            joint_frame = {
                r = {0.5, 0, 0},
            }
        },
        {
            name    = "link2",
            parent  = "link1",
            visuals = {meshes.link},
            body    = bodies.link,
            joint   = joints.rotational_z,
            joint_frame = {
                r = {0,-1, 0},
            }
        },
	{
            name    = "EE",
            parent  = "link2",
            joint_frame = {
                r = {0,-1, 0},
        }
        },


    }

}

return model
