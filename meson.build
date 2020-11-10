project('projectname', 'c',
          default_options : ['b_lto=false', 'b_asneeded=true'])

# Initialize some globals
srcs          = ['src/main.c']
incdirs       = []
c_args        = []
link_args     = []
external_deps = []
rom_address = '0x8000000'
gcc_tools_base = 'arm-none-eabi-'

# enable a lot of warnings
c_args += [
            '-Wall',
            '-Wextra',
            '-Werror',  # warnings are errors
            '-Wshadow', # warns if variable is shadowed
            '-Wdouble-promotion',   # warns if implicit double promotion
            '-Wpedantic',   # strict iso c standard
            '-Wformat=2',   # related to format strings
            '-Wformat-truncation=2',    # related to format strings
            '-Wundef',  # warns of undefined macros
            '-Wconversion', # warns of implicit conversions
            '-Wunused-parameter', # warns of unused function parameters
            '-Wstrict-aliasing', # don't allow pointer aliasing
            '-Wcast-align', # wans if pointer is cast in a way that required alignment is increased
            '-Wpointer-arith', # warns of "sizeof" of function pointers or void*
            '-Wfloat-equal', # warn if floats are used in equality comparison
            '-Wstrict-prototypes', # Warn if a function is declared or defined without specifying the argument types
            '-Winline', # Warn if a function that is declared as inline cannot be inlined
            '-Wnested-externs', # Warn if an extern declaration is encountered within a function
            '-Wunreachable-code',   # Warn if the compiler detects that code will never be executed
            '-Wswitch-default', # Warn whenever a switch statement does not have a default case
            '-Wcast-qual',   # Warn whenever a pointer is cast so as to remove a type qualifier from the target type. For example, warn if a const char * is cast to an ordinary char *
            '-Wmissing-declarations',   # Warn if a global function is defined without a previous declaration. Do so even if the definition itself provides a prototype. Use this option to detect global functions that are not declared in header files
            '-Wmissing-prototypes', # Warn if a global function is defined without a previous prototype declaration. This warning is issued even if the definition itself provides a prototype. The aim is to detect global functions that fail to be declared in header files
            '-Wuninitialized', # Warn if an automatic variable is used without first being initialized
            '-fno-common',  # disables tentative definitions to be merged into a pre-existing definition
            '-fstack-usage',    # generates .su file with stack information (in build/firmware.elf.p)
            '-O2', # speed
          ]

c_args      += [
                 '-ffunction-sections', # generates sections in elf file for each function
                 '-fdata-sections', # generates sectio in elf file for each variable
                 '-g3', # generate debug information
               ]

c_args      += [
  '-DSTM32F411xE',   # for opencm3lib
  '-DSTM32F4',       # for opencm3lib
]

link_args += [
                '-Wl,--gc-sections',  # don't link sections into binary that are not referenced
                '--specs=nosys.specs',
                '-lm'
             ]

#specify linker script(s)
linkfiles   = files(['linker_scripts/link.ld'])

# add external libraries
libopencm3_stm32f4_dep = subproject('libopencm3').get_variable('libopencm3_stm32f4_dep')
external_deps += libopencm3_stm32f4_dep

unity_dep = subproject('unity').get_variable('unity_dep')

# add project sub directories
subdir('src/mpu_6050')
subdir('src/auxiliary')
subdir('src/motor_control')





#=========================================================================================================



# create Debug Flags, depending on meson buildtype
c_args += get_option('buildtype') == 'release' ? '-DNDEBUG' : '-DDEBUG'

# parse linkerfiles to a linker command
link_file_arg = []
if linkfiles != []
    foreach linkfile : linkfiles
    link_file_arg += ['-Wl,-T,@0@/@1@'.format(meson.current_source_dir(), linkfile)]
    endforeach
endif


firmware = executable(
            'firmware',
            [srcs] ,
            name_suffix         : 'elf',
            c_args              : [c_args],
            link_args           : [link_args, link_file_arg],
            dependencies        : [external_deps],
            include_directories : [incdirs],
            build_by_default: false )


# import binary objects
objcopy  = '@0@'.format(find_program('objcopy').path())
objdump  = '@0@'.format(find_program('objdump').path())
size     = '@0@'.format(find_program('size').path())
gdb     = '@0@'.format(find_program('gdb').path())



# these commands should only be executed when compiling for the embedded target
if target_machine.cpu() != 'x86_64'
  # create custom target : create binary, hex dump, size and disassemble dump
  firmware_bin = custom_target(
    'bin',
    output           : ['firmware.bin'],
    build_by_default : false,
    command          : [objcopy, '-O', 'binary', 'firmware.elf', 'firmware.bin'],
    depends          : [firmware]
  )

  firmware_size = custom_target(
    'size',
    capture          : true,
    output           : ['firmware.size'],
    build_by_default : false,
    command          : [size, '--format=berkeley', 'firmware.elf'],
    depends          : [firmware]
  )


  firmware_dump = custom_target(
    'dis',
    capture          : true,
    output           : 'firmware.dis',
    build_by_default : false,
    command          : [objdump, '-D', '-S', '-t', 'firmware.elf'],
    depends          : [firmware]
  )

  run_target(
    'flash',
    command : [
      'st-flash',
      'write',
      '@0@/firmware.bin'.format(meson.current_build_dir()),
      rom_address
    ],
    depends : firmware_bin
  )

  run_target(
    'check_misra',
    command : [
      'cppcheck',
      '--addon=misra',
      '@0@/src'.format(meson.current_source_dir())
    ]
  )

  run_target(
    'check',
    command : [
      'cppcheck',
      '@0@/src'.format(meson.current_source_dir())
    ]
  )

  run_target(
    'puncover',
    command : [
      'puncover',
      '--gcc_tools_base=@0@'.format(gcc_tools_base),
      '--elf_file=@0@/firmware.elf'.format(meson.current_build_dir())
    ],
    depends:firmware
  )

  run_target('bloaty',
    command : [
      'bloaty',
      '@0@/firmware.elf'.format(meson.current_build_dir())
    ],
    depends:firmware
  )

else # target machine is x86_64

  firmware_bin = run_target(
    'bin',
    command : [
      'bash',
      '-c',
      'echo cant do this on x86_64 target machine'
    ]
  )

  firmware_size = run_target(
    'size',
    command : [
      'bash',
      '-c',
      'echo cant do this on x86_64 target machine'
    ]
  )


  firmware_dump = run_target(
    'dis',
    command : [
      'bash',
      '-c',
      'echo cant do this on x86_64 target machine'
    ]
  )

  run_target(
    'flash',
    command : [
      'bash',
      '-c',
      'echo cant do this on x86_64 target machine'
    ]
  )

  run_target(
    'check_misra',
    command : [
      'bash',
      '-c',
      'echo cant do this on x86_64 target machine'
    ]
  )

  run_target(
    'check',
    command : [
      'bash',
      '-c',
      'echo cant do this on x86_64 target machine'
    ]
  )

  run_target(
    'puncover',
    command : [
      'bash',
      '-c',
      'echo cant do this on x86_64 target machine'
    ]
  )

  run_target('bloaty',
    command : [
      'bash',
      '-c',
      'echo cant do this on x86_64 target machine'
    ]
  )
endif