function [Gx_p_handle, Gx_R_handle, Gf_handle] = InitInvObsJacob()

syms stateX stateY stateZ rotX rotY rotZ u v Z
g = BackProjection(stateX, stateY, stateZ, rotX, rotY, rotZ, u, v, Z);

Gx_p_sym = jacobian(g, [stateX, stateY, stateZ]);
Gx_R_sym = jacobian(g, [rotX, rotY, rotZ]);
Gf_sym = jacobian(g, [u, v, Z]);

Gx_p_handle = matlabFunction(Gx_p_sym);
Gx_R_handle = matlabFunction(Gx_R_sym);
Gf_handle = matlabFunction(Gf_sym);

end