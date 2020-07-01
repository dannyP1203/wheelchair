function [dist1, dist2, Errore_relativo, Errore_assoluto] = Errore(X1,Y1,X2,Y2)
%ERRORE Summary of this function goes here
%   Detailed explanation goes here
dist1 = sqrt(X1.^2 + Y1.^2);
dist2 = sqrt(X2.^2 + Y2.^2);
Errore_assoluto = abs(dist1-dist2);
Errore_relativo = Errore_assoluto./mean(dist2);
end

