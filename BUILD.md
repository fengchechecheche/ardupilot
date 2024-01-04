# Building ArduPilot #

Ardupilot is gradually moving from the make-based build system to
[Waf](https://waf.io/). The instructions below should be enough for you to
build Ardupilot, but you can also read more about the build system in the
[Waf Book](https://waf.io/book/).<br> 
Ardupilot正逐渐从基于make的编译系统转向[Waf](https://waf.io/). 
下面的说明应该足以让你编译Ardupilot，但你也可以在[Waf Book](https://waf.io/book/).获取更多信息

Waf should always be called from the ardupilot's root directory. Differently
from the make-based build, with Waf there's a configure step to choose the
board to be used (default is `sitl`).<br> 
应该始终从ardupilot的根目录调用waf。
与基于make的编译不同，Waf有一个配置步骤来选择要使用的飞控板（默认为“sitl”）。

## Basic usage ##

There are several commands in the build system for advanced usages, but here we
list some basic and more used commands as example.<br> 
在编译系统中存在一些高级的命令，但这里我们只列出一些基本的和更常用的命令作为例子。

* **Build ArduCopter**

    Below shows how to build ArduCopter for the Pixhawk2/Cube. Many other boards are
    supported and the next section shows how to get a full list of them.<br>    
    下面展示了如何为Pixhawk2/Cube编译ArduCopter。
    其他飞控板同样支持该操作，下一节将展示如何获得它们的完整列表。

    ```sh
    ./waf configure --board px4-v3
    ./waf copter
    ```

    The first command should be called only once or when you want to change a
    configuration option. One configuration often used is the `--board` option to
    switch from one board to another one. For example we could switch to
    SkyViper GPS drone and build again:<br> 
    第一个命令应该只调用一次，或者当您想要更改配置选项时调用。
    一个经常使用的配置是'--board'选项，从一个飞控板切换到另一个飞控板。
    例如，我们可以切换到SkyViper GPS无人机并重新编译:

    ```sh
    ./waf configure --board skyviper-v2450
    ./waf copter
    ```

    If building for the bebop2 the binary must be built statically:<br> 
    如果编译bebop2，则必须静态编译二进制文件:

    ```sh
    ./waf configure --board bebop --static
    ./waf copter
    ```    

    The "arducopter" and "arducopter-heli" binaries should appear in the `build/<board-name>/bin` directory.<br> 
    “arducopter”和“arducopter-heli”二进制文件应出现在`Build/<board-name>/bin`目录中。

* **List available boards**


    It's possible to get a list of supported boards on ArduPilot with the command
    below<br> 
    可以通过下面的命令获取ArduPilot上受支持的飞控板的列表

    ```sh
    ./waf list_boards

    ```

    Here are some commands to configure waf for commonly used boards:

    ```sh
    ./waf configure --board bebop --static # Bebop or Bebop2
    ./waf configure --board edge           # emlid edge
    ./waf configure --board navio2         # emlid navio2
    ./waf configure --board px4-v1         # the very old two layer Pixhawk (almost none exist)
    ./waf configure --board px4-v2         # older Pixhawks that suffer from the 1MB flash limit issue
    ./waf configure --board px4-v3         # Pixhawk2/Cube and newer Pixhawks with no 1MB flash limit issue
    ./waf configure --board fmuv3          # Pixhawk2/Cube using ChibiOS
    ./waf configure --board px4-v4         # Pixracer
    ./waf configure --board fmuv4          # Pixracer using ChibiOS
    ./waf configure --board skyviper-v2450 # SkyRocket's SkyViper GPS drone using ChibiOS
    ./waf configure --board sitl           # software-in-the-loop simulator
    ./waf configure --board sitl --debug   # software-in-the-loop simulator with debug symbols

    ```

* **Clean the build**

    Commands `clean` and `distclean` can be used to clean the objects produced by
    the build. The first keeps the `configure` information, cleaning only the
    objects for the current board. The second cleans everything for every board,
    including the saved `configure` information.<br> 
    命令“clean”和“distclean”可以用来清理由编译产生的对象。
    命令“clean”会保留 `configure` 选择的飞控板，只清除当前飞控板编译产生的对象。
    命令“distclean”会清除所有飞控板编译产生的所有对象，包括`configure` 选择的飞控板信息。

    Cleaning the build is very often not necessary and discouraged. We do
    incremental builds reducing the build time by orders of magnitude.<br> 
    经常清除编译产生的对象是不必要且不被推荐的。
    我们采用增量式编译，将编译时间成数量级地进行了缩短。


* **Upload or install**

    Build commands have a `--upload` option in order to upload the binary built
    to a connected board. This option is supported by Pixhawk and Linux-based boards.
    The command below uses the `--targets` option that is explained in the next item.<br> 
    编译命令有一个 `--upload` 选项，来将生成的二进制文件上传到连接的飞控板。
    Pixhawk和Linux的飞控板均支持这个选项。
    下方的编译命令使用了 `--targets` 选项，这个选项将在下一节进行介绍。

    ```sh
    ./waf --targets bin/arducopter --upload
    ```

    For Linux boards you need first to configure the IP of the board you
    are going to upload to. This is done on configure phase with:<br> 
    在基于Linux的飞控板中上传二进制文件前，需要先配置飞控板的IP地址。
    配置命令如下方语句所示：

    ```sh
    ./waf configure --board <board> --rsync-dest <destination>
    ```

    The commands below give a concrete example (board and destination
    IP will change according to the board used):<br> 
    下方命令给出了一个具体的例子（所选择的飞控板和目标IP地址根据实际使用的飞控板不同而不同）：

    ```sh
    ./waf configure --board navio2 --rsync-dest root@192.168.1.2:/
    ./waf --target bin/arducopter --upload
    ```

    This allows to set a destination to which the `--upload` option will upload
    the binary.  Under the hood  it installs to a temporary location and calls
    `rsync <temp_install_location>/ <destination>`.<br> 
    上方代码允许设置一个 `--upload` 选项上传二进制文件的目标地址。
    进一步地，这个二进制文件也可以通过调用`rsync <temp_install_location>/ <destination>`安装到一个临时位置。
    

    On Linux boards there's also an install command, which will install to a certain
    directory, just like the temporary install above does. This can be
    used by distributors to create .deb, .rpm or other package types:<br> 
    在基于Linux的飞控板上，和上面安装二进制文件到临时位置的命令一样，
    同样有一个安装命令将二进制文件安装到一个确定的位置。
    开发者可以使用它来创建.deb、.rpm或其他程序包类型：

    ```sh
    ./waf copter
    DESTDIR=/my/temporary/location ./waf install
    ```

* **Use different targets**

    The build commands in the items above use `copter` as argument. This
    builds all binaries that fall under the "copter" group. See the
    section [Advanced usage](#advanced-usage) below for more details regarding
    groups.<br> 
    以上各项中的编译命令使用`copter`作为参数。
    这将编译属于“Copter”组的所有二进制文件。
    有关组的更多详细信息，请参阅下面的[Advanced usage](#advanced-usage) 一节。

    This shows a list of all possible targets:<br> 
    下方命令将显示所有可以编译的目标

    ```
    ./waf list
    ```

    For example, to build only a single binary:<br> 
    例如，只编译某一个二进制文件：

    ```
    # Quad frame of ArduCopter
    ./waf --targets bin/arducopter

    # unit test of our math functions
    ./waf --targets tests/test_math
    ```

* **Other options**

    It's possible to see all available commands and options:<br>
    通过下方命令可以查看所有支持的命令的选项：

    ```
    ./waf -h
    ```

    Also, take a look on the [Advanced section](#advanced-usage) below.<br
    更多详细信息，请参阅下面的[Advanced usage](#advanced-usage) 一节。>

## Advanced usage ##

This section contains some explanations on how the Waf build system works
and how you can use more advanced features.<br>
本节包含了一些 Waf 编译系统如何工作，以及如何使用高级功能的例子。

Waf build system is composed of commands. For example, the command below
(`configure`) is for configuring the build with all the options used by this
particular build.<br>
Waf 编译系统由众多命令组成。
例如，下方命令用于配置实际编译过程中的所有选项。

```bash
# Configure the Linux board
./waf configure --board=linux
```

Consequently, in order to build, a "build" command is issued, thus `waf build`.
That is the default command, so calling just `waf` is enough:<br>
因此，为了进行编译，会发出一个 "build" 命令，即`waf build`。
这是默认命令，只调用`waf`就足够了：

```bash
# Build programs from bin group
./waf

# Waf also accepts '-j' option to parallelize the build.
./waf -j8
```

By default waf tries to parallelize the build automatically to all processors
so the `-j` option is usually not needed, unless you are using icecc (thus
you want a bigger value) or you don't want to stress your machine with
the build.<br>
默认情况下，waf 尝试将编译自动并行化到所有处理器，因此通常不需要`-j`选项，
除非您正在使用icecc(因此您想要更大的值)，或者您不想为编译操作分配电脑的所有算力。

### Program groups ###

Program groups are used to represent a class of programs. They can be used to
build all programs of a certain class without having to specify each program.
It's possible for two groups to overlap, except when both groups are main
groups. In other words, a program can belong to more than one group, but only
to one main group.<br>
程序组用于表示一类程序。
它们可以用来构建某一类的所有程序，而不必指定每个程序。
两个组有可能重叠，除非两个组都是主要组。
换句话说，一个程序可以属于多个组，但只能属于一个主组。

There's a special group, called "all", that comprises all programs.<br>
有一个特殊的组，叫做"all"，由所有的程序组成。

#### Main groups ####

The main groups form a partition of all programs. Besides separating the
programs logically, they also define where they are built.<br>
main 组构成了所有程序的分区。
除了在逻辑上区分程序之外，它们还定义了它们的编译位置。

The main groups are:

 - bin: *the main binaries, that is, ardupilot's main products - the vehicles and
   Antenna Tracker*
 - tools
 - examples: *programs that show how certain libraries are used or to simply
   test their operation*
 - benchmarks: *requires `--enable-benchmarks` during configurarion*
 - tests: *basically unit tests to ensure changes don't break the system's
   logic*

All build files are placed under `build/<board>/`, where `<board>` represents
the board/platform you selected during configuration. Each main program group
has a folder with its name directly under `build/<board>/`. Thus, a program
will be stored in `build/<board>/<main_group>/`, where `<main_group>` is the
main group the program belongs to. For example, for a linux build, arduplane,
which belongs to the main group "bin", will be located at
`build/linux/bin/arduplane`.

#### Main product groups ####

Those are groups for ardupilot's main products. They contain programs for the
product they represent. Currently only the "copter" group has more than one
program - one for each frame type.

The main product groups are:

 - antennatracker
 - copter
 - plane
 - rover

#### Building a program group ####

Ardupilot adds to waf an option called `--program-group`, which receives as
argument the group you want it to build. For a build command, if you don't pass
any of `--targets` or `--program-group`, then the group "bin" is selected by
default. The option `--program-group` can be passed multiple times.

Examples:

```bash
# Group bin is the default one
./waf

# Build all vehicles and Antenna Tracker
./waf --program-group bin

# Build all benchmarks and tests
./waf --program-group benchmarks --program-group tests
```
#### Shortcut for program groups ####

For less typing, you can use the group name as the command to waf. Examples:

```bash
# Build all vehicles and Antenna Tracker
./waf bin

# Build all examples
./waf examples

# Build arducopter binaries
./waf copter
```

### Building a specific program ###

In order to build a specific program, you just need to pass its path relative
to `build/<board>/` to the option `--targets`. Example:

```bash
# Build arducopter for quad frame
./waf --targets bin/arducopter

# Build vectors unit test
./waf --targets tests/test_vectors
```

### Checking ###

The command `check` builds all programs and then executes the relevant tests.
In that context, a relevant test is a program from the group "tests" that makes
one of the following statements true:

 - it's the first time the test is built since the last cleanup or when the
   project was cloned.
 - the program had to be rebuilt (due to modifications in the code or
   dependencies, for example)
 - the test program failed in the previous check.

That is, the tests are run only if necessary. If you want waf to run all tests,
then you can use either option `--alltests` or the shortcut command
`check-all`.

Examples:

```bash
# Build everything and run relevant tests
./waf check

# Build everything and run all tests
./waf check --alltests

# Build everything and run all tests
./waf check-all
```

### Debugging ###

It's possible to pass the option `--debug` to the `configure` command. That
will set compiler flags to store debugging information in the binaries so that
you can use them with `gdb`, for example. That option might come handy when using SITL.

### Build-system wrappers ###

The `waf` binary on root tree is actually a wrapper to the real `waf` that's
maintained in its own submodule.  It's possible to call the latter directly via
`./modules/waf/waf-light` or to use an alias if you prefer typing `waf` over
`./waf`.

```sh
alias waf="<ardupilot-directory>/modules/waf/waf-light"

```

There's also a make wrapper called `Makefile.waf`. You can use
`make -f Makefile.waf help` for instructions on how to use it.

### Command line help ###

You can use `waf --help` to see information about commands and options built-in
to waf as well as some quick help on those added by ardupilot.
