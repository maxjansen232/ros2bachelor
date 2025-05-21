import capytaine as cpt
from numpy import inf
cpt.set_logging("WARNING")
#rov is set as a box with dimensions 4x1.35x0.67 m
rov = cpt.mesh_parallelepiped(size=(4, 1.35, 0.67), center = (0,0,0))
#body is based on mesh of ROV
body = cpt.FloatingBody(mesh=rov,
                        dofs=cpt.rigid_body_dofs(rotation_center=(0,0,0)),
                        center_of_mass=(0,0,0))

#Setup of the problems to solve with BEMsolver: 
problem_x = cpt.RadiationProblem(body=body, radiating_dof="Surge", omega=inf, rho=1000, water_depth=inf, free_surface=inf)
problem_y = cpt.RadiationProblem(body=body, radiating_dof="Sway", omega=inf, rho=1000, water_depth=inf, free_surface=inf)
problem_z = cpt.RadiationProblem(body=body, radiating_dof="Heave", omega=inf, rho=1000, water_depth=inf, free_surface=inf)

solver1 = cpt.BEMSolver()

#Solve for added mass in X(Surge), Y(Sway) and Z(Heave) 
xresult = solver1.solve(problem_x)
yresult = solver1.solve(problem_y)
zresult = solver1.solve(problem_z)

#Print results of calculations
print(xresult.added_masses)
print(yresult.added_masses)
print(zresult.added_masses)
rov.show()