import Sofa

USE_GUI = True

def main():
    import SofaRuntime
    import Sofa.Gui

    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)

    if not USE_GUI:
        for iteration in range(10):
            Sofa.Simulation.animate(root, root.dt.value)
            print("Tempo corrente (senza GUI):", root.time.value)  # Leggi il tempo durante la simulazione
    else:
        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()

def createScene(root):
    root.gravity = [0, -9.81, 0]
    root.dt = 0.02
    root.addObject("RequiredPlugin", pluginName=[
        'Sofa.Component.Collision.Detection.Intersection',
        'Sofa.Component.Collision.Detection.Algorithm',
        'Sofa.Component.Collision.Geometry',
        'Sofa.Component.Collision.Response.Contact',
        'Sofa.Component.Constraint.Projective',
        'Sofa.Component.IO.Mesh',
        'Sofa.Component.LinearSolver.Iterative',
        'Sofa.Component.Mapping.Linear',
        'Sofa.Component.Mass',
        'Sofa.Component.ODESolver.Backward',
        'Sofa.Component.SolidMechanics.FEM.Elastic',
        'Sofa.Component.StateContainer',
        'Sofa.Component.Topology.Container.Dynamic',
        'Sofa.Component.Visual',
        'Sofa.GL.Component.Rendering3D'
    ])
    root.addObject('DefaultAnimationLoop')

    root.addObject('VisualStyle', displayFlags="showCollisionModels")
    root.addObject('CollisionPipeline', name="CollisionPipeline")
    root.addObject('BruteForceBroadPhase', name="BroadPhase")
    root.addObject('BVHNarrowPhase', name="NarrowPhase")
    root.addObject('CollisionResponse', name="CollisionResponse", response="PenalityContactForceField")
    root.addObject('DiscreteIntersection')

    root.addObject('MeshOBJLoader', name="LiverSurface", filename="mesh/liver-smooth.obj")

    liver = root.addChild('Liver')
    liver.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
    liver.addObject('CGLinearSolver', name="linear_solver", iterations="25", tolerance="1e-09", threshold="1e-09")
    liver.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/liver.msh")
    liver.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
    liver.addObject('MechanicalObject', name="dofs", src="@meshLoader")
    liver.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    liver.addObject('DiagonalMass', name="Mass", massDensity="1.0")
    liver.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio="0.3", youngModulus="3000", computeGlobalMatrix="0")
    liver.addObject('FixedProjectiveConstraint', name="FixedConstraint", indices="3 39 64")

    visu = liver.addChild('Visu')
    visu.addObject('OglModel', name="VisualModel", src="@../../LiverSurface")
    visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")

    surf = liver.addChild('Surf')
    surf.addObject('SphereLoader', name="sphereLoader", filename="mesh/liver.sph")
    surf.addObject('MechanicalObject', name="spheres", position="@sphereLoader.position")
    surf.addObject('SphereCollisionModel', name="CollisionModel", listRadius="@sphereLoader.listRadius")
    surf.addObject('BarycentricMapping', name="CollisionMapping", input="@../dofs", output="@spheres")

    root.addObject('MeshOBJLoader', name='Loader-torus', filename='mesh/torus2_scale3.obj')

    torus = root.addChild('Torus')
    torus.addObject('EulerImplicitSolver', name="cg_odesolver", printLog="false", rayleighStiffness="0.1", rayleighMass="0.1")
    torus.addObject('CGLinearSolver', iterations="25", name="linear solver", tolerance="1.0e-9", threshold="1.0e-9")
    torus.addObject('MechanicalObject', dx="0", dy="25", dz="0", rz="64")
    torus.addObject('UniformMass', totalMass="10")
    torus.addObject('RegularGridTopology', nx="6", ny="5", nz="2", xmin="-7.5", xmax="7.5", ymin="-6", ymax="6", zmin="-1.75", zmax="1.75")
    torus.addObject('RegularGridSpringForceField', name="Springs", stiffness="350", damping="1")

    visu = torus.addChild('Visu')
    visu.addObject('OglModel', name="Visual", src="@../../Loader-torus", color="blue")
    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

    surf = torus.addChild('Surf')
    surf.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_scale3.obj")
    surf.addObject('MeshTopology', src="@loader")
    surf.addObject('MechanicalObject', name="spheres", position="@[-1].position")
    surf.addObject('SphereCollisionModel', name="CollisionModel", radius="0.6")
    surf.addObject('BarycentricMapping')

    return root

if __name__ == '__main__':
    main()

