function angle = calculate_angle(A, B, C)
% Calculate vectors AB and BC in 3D
AB = A - B;
BC = C - B;

% Check if any two points are the same
if all(round(A,6) == round(B,6)) || all(round(B,6) == round(C,6))
    angle = 180;
else
    % Calculate dot product and magnitudes of the vectors
    dotProduct = dot(AB, BC);
    magAB = norm(AB);
    magBC = norm(BC);

    % Calculate the cosine of the angle between AB and BC
    cosTheta = dotProduct / (magAB * magBC);

    % Ensure the cosine value is within the valid range for acos
    cosTheta = max(min(cosTheta, 1), -1);

    % Calculate the angle in radians and convert to degrees
    angle = acosd(cosTheta);
end
end
