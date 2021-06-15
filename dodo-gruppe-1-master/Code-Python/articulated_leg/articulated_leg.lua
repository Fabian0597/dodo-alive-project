-- Articulated Leg 

-- Body properties
length_link1 = 0.8
length_link2 = 0.8
width_link1 = 0.05
width_link2 = 0.05

m_link1 = 10.
m_link2 = 20.
m_base = 20.

I_link1_xx = (1/12)*m_link1*width_link1*width_link1
I_link1_zz = (1/12)*m_link1*length_link1*length_link1
I_link2_xx = (1/12)*m_link2*width_link2*width_link2
I_link2_zz = (1/12)*m_link2*length_link2*length_link2

radius_floating_base = 0.2
radius_foot = 0.1

-- Matrix definitions
inertiaMatrix_link1 = {
    {I_link1_xx     ,0.     ,0.    },
    {0.    ,I_link1_xx      ,0.    },
    {0.    ,0.          ,I_link1_zz}        
}

inertiaMatrix_link2 = {
    {I_link2_xx     ,0.     ,0.    },
    {0.    ,I_link2_xx      ,0.    },
    {0.    ,0.          ,I_link2_zz}        
}

nullMatrix = {
    {0.    ,0.     ,0.    },
    {0.    ,0.     ,0.    },
    {0.    ,0.     ,0.    }        
}

-- Geometry properties
link1Properties= {   mass    = m_link1, 
                    com     = {0.,-length_link1/2,0.}, 
                    inertia = inertiaMatrix_link1}
link2Properties= {   mass    = m_link2, 
                    com     = {0.,-length_link2/2,0.}, 
                    inertia = inertiaMatrix_link2}
baseProperties= {   mass    = m_base, 
                    com     = {0.,0.,0.}, 
                    inertia = nullMatrix}
footProperties= {   mass    = 0.1, 
                    com     = {0.,0.,0.}, 
                    inertia = nullMatrix}

-- Putting bodies in a table
bodies = {
    link1  = link1Properties,
    link2 = link2Properties,
    foot = footProperties,
    floatingBase  = baseProperties}

-- Making a table of joints
joints = {
    rotational_z = {
        {0.,0.,1.,0.,0.,0.}   
    },
    floating = {
        { 0., 0., 0., 1., 0., 0.},
        { 0., 0., 0., 0., 1., 0.}
    }
}
-- Making the meshes
meshes = {
    link1 = {
        dimensions = { width_link1, length_link1, width_link1 },
        color = { 1, 0, 0 },
        mesh_center = { 0, -length_link1/2, 0 },
        src = "unit_cube.obj",
    },
    link2 = {
        dimensions = { width_link2, length_link2, width_link2 },
        color = { 0, 0, 1 },
        mesh_center = { 0, -length_link2/2, 0 },
        src = "unit_cube.obj",
    },
    floatingBase = {
        dimensions = {radius_floating_base, radius_floating_base, radius_floating_base},
        color       = { 0, 1, 0 },
        mesh_center = { 0, 0.0, 0 },
        src = "unit_sphere_medres.obj",
    },
    foot = {
        dimensions = {radius_foot, radius_foot, radius_foot},
        color       = { 1, 0, 0 },
        mesh_center = { 0, 0, 0 },
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
            visuals = {meshes.link1},
            body    = bodies.link1,
            joint   = joints.rotational_z,
            joint_frame = {
                r = {0, 0, 0},
            }
        },
        {
            name    = "link2",
            parent  = "link1",
            visuals = {meshes.link2},
            body    = bodies.link2,
            joint   = joints.rotational_z,
            joint_frame = {
                r = {0,-length_link1, 0},
            }
        },
        {
            name    = "foot",
            parent  = "link2",
            visuals = {meshes.foot},
            body    = bodies.foot,
            joint_frame = {
                r = {0, -length_link2, 0},
            }
        },


    }

}

return model
