def precondition_path(path: list) -> list:
    if len(path) >= 2 and path[1][4] == 1:
        path[0][4] = 1

    i = 1
    pop_count = 0
    pop_threshold = 15

    while i < len(path) - 1:

        if path[i + 1][4] == 1:
            path[i][4] = 1

        if (i > 1
                and abs(path[i - 1][2] - path[i][2]) < 1e-4
                and abs(path[i][2] - path[i + 1][2]) < 1e-4
                and pop_count < pop_threshold
                and path[i][4] == 0):
            path.pop(i)
            pop_count += 1
        else:
            i += 1
            pop_count = 0

    return path
