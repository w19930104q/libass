test_src = files('test.c')

libass_test = executable(
    'libass_test',
    test_src,
    install: false,
    include_directories: incs,
    dependencies: deps + png_deps,
    link_with: libass_for_tools,
)

drawing_test_src = files('test_drawing.c')

libass_drawing_test = executable(
    'drawing_test',
    drawing_test_src,
    install: false,
    include_directories: incs,
    dependencies: deps,
    link_with: libass_for_tools,
)

test('drawing', libass_drawing_test)
