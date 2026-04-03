% *************************************************************************
% Function to compute moments of a set of feature points nP
% *************************************************************************
function m_ij = moment(nP, i, j)
    m_ij = sum(nP(1,:).^i.*nP(2,:).^j);
end