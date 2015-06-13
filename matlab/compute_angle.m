function angl = compute_angle(from, to)
    deltax = to(1) - from(1);
    deltay = to(2) - from(2);
    angl = atan2(deltay, deltax);
end