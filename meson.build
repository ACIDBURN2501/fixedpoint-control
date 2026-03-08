project('fixedpoint-control', 'c',
  version : '0.1.0',
  default_options : ['c_std=c11'])

conf = configuration_data()
conf.set('VERSION', meson.project_version())
version_header = configure_file(
  input : 'config/fpc_version.h.in',
  output : 'fpc_version.h',
  configuration : conf,
  install : false)

fpc_build_conf = configuration_data()
fpc_max_instances = get_option('fpc_max_instances')
fpc_pool_item_size = get_option('fpc_pool_item_size')
fpc_build_conf.set('FPC_MAX_INSTANCES', fpc_max_instances)
fpc_build_conf.set('FPC_POOL_ITEM_SIZE', fpc_pool_item_size)
build_config_header = configure_file(
  input : 'config/fpc_build_config.h.in',
  output : 'fpc_build_config.h',
  configuration : fpc_build_conf,
  install : false)

inc = include_directories('include', '.')

# Pool allocator subproject
# Note: pool_item_size must be a multiple of max_align_t (16 on this system)
pool_subproj = subproject('pool-allocator',
  default_options : [
    'build_tests=false',
    'lookup_strategy=linear',
    'pool_max_slots=@0@'.format(fpc_max_instances),
    'pool_item_size=@0@'.format(fpc_pool_item_size)
  ]
)
pool_lib = pool_subproj.get_variable('pool_lib')
pool_dep = declare_dependency(
  include_directories : pool_subproj.get_variable('inc'))

subdir('src')

fpc_dep = declare_dependency(
  link_with : lib,
  include_directories : inc)

meson.override_dependency('fixedpoint-control', fpc_dep)

install_headers(
  files(
    'include/fpc_status.h',
    'include/fpc_config.h',
    'include/fpc_pool_config.h',
    'include/fpc_pid.h',
    'include/fpc_filters.h'
  ),
  version_header,
  build_config_header,
  subdir : 'fixedpoint-control'
)

subdir('tests')

pkg = import('pkgconfig')
pkg.generate(
  lib,
  name : 'fixedpoint-control',
  filebase : 'fixedpoint-control',
  description : 'Safety-oriented fixed-point PID and filter library',
  subdirs : 'fixedpoint-control'
)
