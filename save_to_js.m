function save_to_js(filenames, colors, sizes, scales)
    if (length(filenames) ~= length(colors)) || (length(filenames) ...
                                                 ~= length(colors)) || (length(filenames) ~= length(scales))
        fprintf('Argument lengths not equal!\n');
        return;
    end
    fd = fopen('data.js', 'w');
    fprintf(fd, 'var data = [\n');
    for i = 1:length(filenames)
        vars = load(filenames{i});
        tmp = fieldnames(vars);
        var_name = tmp{1};
        var = getfield(vars, var_name);
        fprintf(fd, '{color:"%s",size:%d,points:[\n', colors{i}, sizes(i));
        for j = 1:length(var)
            fprintf(fd, '"%.6f|%.6f",\n', var(j, 1)*scales(i), var(j, 2)*scales(i));
        end
        fprintf(fd, ']},\n');
    end
    fprintf(fd, '];\n');
    fclose(fd);
end