function[Points]=world_to_image(f,worldPoints,R,t)

A = -f *((R(1,1)*(worldPoints(:,1)-t(1)) + R(1,2)*(worldPoints(:,2)-t(2)) + R(1,3)*(worldPoints(:,3)-t(3)))...
    ./(R(3,1)*(worldPoints(:,1)-t(1)) + R(3,2)*(worldPoints(:,2)-t(2)) + R(3,3)*(worldPoints(:,3)-t(3))));
B = -f *((R(2,1)*(worldPoints(:,1)-t(1)) + R(2,2)*(worldPoints(:,2)-t(2)) + R(2,3)*(worldPoints(:,3)-t(3)))...
    ./(R(3,1)*(worldPoints(:,1)-t(1)) + R(3,2)*(worldPoints(:,2)-t(2)) + R(3,3)*(worldPoints(:,3)-t(3))));

Points=[A,B];

end