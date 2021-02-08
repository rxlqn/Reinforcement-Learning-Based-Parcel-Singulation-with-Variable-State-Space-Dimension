swig -python example.i
make

以上是example.i文件。

注意：

%module指定了模块名为example。
%{}%则指定了在生成的example_wrap.c文件中需要的声明，这里用了一个example.h文件。 如果换成下面的四行extern，在这个Demo里也是等效的。
extern的四行，声明了需要对外暴露的四个接口。
除了%部分，其它均遵循.h文件的语法。
虽然example.h的内容和四个extern几乎是一致的，而且%{}%中也可以直接使用extern，但第三部分不能使用#include。