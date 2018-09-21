def convertPLYToPCD(mesh_file, pcd_file):
    input = open(mesh_file)
    out = pcd_file
    output = open(out, 'w')
    write_points = False
    points_counter = 0
    nr_points = 0
    for s in input.xreadlines():
        if s.find("element vertex") != -1:
            nr_points = int(s.split(" ")[2].rstrip().lstrip())
            new_header = header.replace("XXXX", str(nr_points))
            output.write(new_header)
            output.write("\n")
        if s.find("end_header") != -1:
            write_points = True
            continue
        if write_points and points_counter < nr_points:
            points_counter = points_counter + 1
            output.write(" ".join(s.split(" ", 3)[:3]))
            output.write("\n")
    input.close()
    output.close()