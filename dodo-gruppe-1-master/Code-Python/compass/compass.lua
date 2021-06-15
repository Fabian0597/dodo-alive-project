-- Compass gait robot

-- Matrix definitions
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

-- Body mass and geometry properties
linkProperties= {   mass    = 1., 
                    com     = {0.,-1.,0.}, 
                    inertia = inertiaMatrix}
baseProperties= {   mass    = 1., 
                    com     = {0.,0.,0.}, 
                    inertia = nullMatrix}
footProperties= {   mass    = 0.001, 
                    com     = {0.,0.,0.}, 
                    inertia = nullMatrix}

-- Putting bodies in a table
bodies = {
    link  = linkProperties,
    floatingBase  = baseProperties}

-- Making a table of joints
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
-- Making the meshes
meshes = {
    link = {
        dimensions = { 0.05, 0.95, 0.05 },
        color = { 1, 0, 0 },
        mesh_center = { 0, -0.5, 0 },
        src = "unit_cube.obj",
    },
    floatingBase = {
        dimensions = {0.2,0.2,0.2},
        color       = { 0, 1, 0 },
        mesh_center = { 0, 0.0, 0 },
        src = "unit_sphere_medres.obj",
    },
    foot = {
        dimensions = {0.1,0.1,0.1},
        color       = { 1, 0, 0 },
        mesh_center = { 0, 0.05, 0 },
        src = "unit_sphere_medres.obj",
    }
}

-- Making the model
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
            joint_frame = {
                r = {0, 0, 0},
            }
        },
        {
            name    = "foot1",
            parent  = "link1",
            visuals = {meshes.foot},
            body    = bodies.foot,
            joint_frame = {
                r = {0, -1.0, 0},
            }
        },
        {
            name    = "link2",
            parent  = "floatingBase",
            visuals = {meshes.link},
            body    = bodies.link,
            joint   = joints.rotational_z,
            joint_frame = {
                r = {0,0, 0},
            }
        },
        {
            name    = "foot2",
            parent  = "link2",
            visuals = {meshes.foot},
            body    = bodies.foot,
            joint_frame = {
                r = {0, -1.0, 0},
            }
        },


    }

}

return model
