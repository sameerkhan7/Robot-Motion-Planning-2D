function vertices = get_vertices(lengths, q)

    vertices = [];              % columns of 3D points
    vertices(:,1) = [0;0;0;1];

    g01 = [ROTZ(q(1)), [0;0;lengths(1)]; 0 0 0 1];
    vertices(:,2) = g01*vertices(:,1);

    g12 = [ROTX(q(2)), [lengths(2);0;0]; 0 0 0 1];
    vertices(:,3) = g01*g12*vertices(:,1);

    g23 = [ROTZ(q(3)), [0;0;lengths(3)]; 0 0 0 1];
    vertices(:,4) = g01*g12*g23*vertices(:,1);

    g34 = [ROTX(q(4)), [-lengths(4);0;0]; 0 0 0 1];
    vertices(:,5) = g01*g12*g23*g34*vertices(:,1);

    g45 = [ROTZ(q(5)), [0;0;lengths(5)]; 0 0 0 1];
    vertices(:,6) = g01*g12*g23*g34*g45*vertices(:,1);

    g56 = [ROTX(q(6)), [lengths(6);0;0]; 0 0 0 1];
    vertices(:,7) = g01*g12*g23*g34*g45*g56*vertices(:,1);

    g67 = [ROTZ(q(7)), [0;0;lengths(7)]; 0 0 0 1];
    vertices(:,8) = g01*g12*g23*g34*g45*g56*g67*vertices(:,1);

    g78 = [eye(3), [lengths(8);0;0]; 0 0 0 1];
    vertices(:,9) = g01*g12*g23*g34*g45*g56*g67*g78*vertices(:,1);

end