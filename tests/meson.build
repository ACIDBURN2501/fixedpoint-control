test_pid_src = files('test_pid.c')
test_filters_src = files('test_filters.c')

test_exe_pid = executable('test_pid',
        test_pid_src,
        include_directories : inc,
        link_with : lib)

test_exe_filters = executable('test_filters',
        test_filters_src,
        include_directories : inc,
        link_with : lib)

test('pid_controller', test_exe_pid)
test('filters', test_exe_filters)
