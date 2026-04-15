% *************************************************************************
% Function to compute centered moments of a set of feature points nP
% *************************************************************************
function mu_ij = centered_moment(nP, P_g, i, j)
    dP = nP - P_g;
    mu_ij = sum(dP(1,:).^i.*dP(2,:).^j);
end