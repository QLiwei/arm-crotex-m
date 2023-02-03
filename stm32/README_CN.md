# 1.开发环境

## 1.1 常用工具

### 1.1.1 集成开发环境（IDE）

1. Keil公司的Real View MDK
2. IAR公司的EWARM

### 1.1.2 源文件编辑器

1. [UltraEdit](https://www.ultraedit.com/)
2. [SourceInsight](https://www.sourceinsight.com/)
3. [Visual Studio Code](https://code.visualstudio.com/)

### 1.1.3 仿真器

1. [J-Link](https://www.segger.com/products/debug-probes/j-link/)
2. [U-Link](https://www2.keil.com/mdk5/ulink/ulink2/)
3. [ST-Link](https://www.st.com/en/development-tools/st-link-v2.html)
4. [CMSIS-DAP](https://os.mbed.com/handbook/CMSIS-DAP)

### 1.1.4 串口工具

1. [SecureCRT](https://www.vandyke.com/products/securecrt/)
2. [sscom](http://www.sscom.vip/)

### 1.1.5 CH340驱动

下载[CH340](https://www.wch.cn/download/CH341SER_EXE.html) 

## 1.2 MDK

### 1.2.1 MDK下载

1. 官网下载：[arm keil](https://www.keil.com/download/product/) 
2. 论坛下载：[armbbs](https://www.armbbs.cn/forum.php?mod=viewthread&tid=89403)

安装不要设置中文安装目录

### 1.2.2 Keil软件包下载

1. 官网下载：[software-pack](https://www.keil.com/dd2/Pack/) 
2. Keil软件Pack Installer安装

### 1.2.3 MDK注册

## 1.3 IAR

### 1.3.1 IAR下载

1. 官网下载：[IAR](https://www.iar.com/products#/search?architecture=Arm)

# 2.KEIL调试

# 3.Event Recorder

# 4.数据类型、变量和堆栈

## 4.1.变量

### 4.1.1局部变量

函数内部定义的变量，只在函数内有效

- 形式参数也是局部变量
- 不同函数相同变量互不干扰
- 作用域在函数内部

### 4.1.2全局变量

定义在函数之外的变量

- 有效范围从定义变量位置开始到源文件结束
- 同一个源文件 外部变量和局部变量同名，外部变量被屏蔽

### 4.1.3使用全局变量缺点

- 全局变量在程序的执行过程中都占用存储单元，而不是仅在需要时才占用存储单元。
- 函数的通用性降低了，因为函数在执行时要依赖于其所在的外部变量。如果将一个函数移植到另一个 文件中，还要将有关的外部变量及其值一起移植过去。
- 使用全局变量过多，会降低程序的清晰性，特别是多个函数都调用此变量时。

### 4.1.4变量的存储类别

- 静态存储方式：在程序运行期间由系统分配固定的存储方式
- 动态存储方式：在程序运行期间根据需要进行动态的分配存储空间方式

动态存储区：

- 函数的形参，在调用时给形参分配存储空间
- 局部变量（未加 static 声明的局部变量）。
- 函数调用时的现场保护和返回地址等。

### 4.1.5static

- 对局部变量用 static 声明，则使用该变量在整个程序执行期间不释放，为其分配的的空间始终存在
- 全局变量用 static 声明，则该变量的作用域只限于本文件模块（即被声明的文件中）。

## 4.2.堆栈

栈（stack）空间：用于局部变量，函数调用时现场保护和返回地址，函数的形参

堆（heap）空间：主要应用动态内存分配，malloc calloc realloc

### 4.2.1寄存器组（堆栈指针寄存器）

Cortex – M7/M4/M3 处理器拥有 R0-R15 的通用寄存器组。其中 R13 作为堆栈指针 SP。SP 有两 个，但在同一时刻只能有一个可以用。

- 主堆栈指针（MSP）：这是缺省的堆栈指针，它由 OS 内核、异常服务例程以及所有需要特权访问的 应用程序代码来使用。
- 进程堆栈指针（PSP）：用于常规的应用程序代码（不处于异常服务例程中时）。
- 大多数情况下的应用，只需使用指针 MSP，而 PSP 多用于 RTOS 中。
- R13 的最低两位被硬线连接到 0，并且总是读出 0，这意味着堆栈总是 4 字节对齐的。

```
R0-R12			通用寄存器
R13(MSP,PSP)	堆栈指针SP
R14				连接寄存器LR
R15				程序计数器PC
```

**LR寄存器**：当通过BL或BLX指令调用[子程序](https://baike.baidu.com/item/子程序?fromModule=lemma_inlink)时，硬件自动将子程序返回地址保存在R14[寄存器](https://baike.baidu.com/item/寄存器?fromModule=lemma_inlink)中。在[子程序](https://baike.baidu.com/item/子程序?fromModule=lemma_inlink)返回时，把LR的值复制到[程序计数器](https://baike.baidu.com/item/程序计数器?fromModule=lemma_inlink)PC即可实现子程序返回

**PC 寄存器**：用来存储指向下一条指令的地址，也即将要执行的指令代码。由执行引擎读取下一条指令。

### 4.2.2Cortex-M7/M4/M3 向下生长的满栈

```

```

### 4.2.3堆栈基本操作

```
PUSH 入栈 SP先自减4，在存入新的数值
R0 0x12345678
-----------------------
高地址		  |	已使用
			| 已使用
	   	 	| 最近一次压入的数据  <- SP
	   	 	|	  -
低地址		  |		-
PUSH R0
-----------------------
高地址		  |	已使用
			| 已使用
	   	 	| 已使用  
	   	 	| 0x12345678		<- SP 向下生长
低地址		  |		-


POP 出栈	先读出SP指针处数组，SP再自增4
-----------------------
高地址		  |	已使用
			| 已使用
	   	 	| 已使用  
	   	 	| 0x12345678		<- SP 向下生长
低地址		  |		-

POP R0
-----------------------
高地址		  |	已使用
			| 已使用
	   	 	| 已使用  				<- SP
	   	 	| 	-
低地址		  |   -

R0 = 0x12345678
```

### 4.2.4局部变量，全局变量和堆栈

```c
uint32_t a = 0; //全局初始化区, 可以被其他 c 文件 extern 引用
static uint32_t ss = 0; //静态变量，只允许在本文件使用
uint8_t *p1; //全局未初始化区
int main(void)
{
	uint32_t b; //栈
	uint8_t s[] = "abc"; //栈
	uint8_t *p2; //栈
	uint8_t *p3 = "123456"; //123456\0 在常量区，p3 在栈上。
	static uint32_t c =0; //全局（静态）初始化区
	p1 = (uint8_t *)malloc(10); //在堆区申请了 10 个字节空间
	p2 = (uint8_t *)malloc(20); //在堆区申请了 20 个字节空间
	strcpy(p1, "123456"); /* 123456 字符串（结束符号是 0(\0)，总长度 7）放在常量区，编译器可能会将它与 p3 所指向的"123456"优化成一个地方 */
}
```



# 5.MAP文件分析

## 5.1.MAP文件分析

**MAP文件内容可分为已下几个部分：**

- Section Cross References：不同文件中函数的调用关系
- Removing Unused input sections from the image：是被删除的冗余函数
- Image Symbol Table (Local Symbols Global Symbols)
- Memory Map of the image
- Image component sizes

## 5.2 MAP文件相关概念

- 段(section) ：描述映像文件的代码和数据块。
- RO： Read-Only 的缩写，包括 RO-data(只读数据)和 RO-code(代码)。
- RW：Read-Write 的缩写，主要是 RW-data，RW-data 由程序初始化初始值。
- ZI： Zero-initialized 的缩写，主要是 ZI-data，由编译器初始化为 0。
- .text：与 RO-code 同义。
- .constdata：与 RO-data 同义。
- .bss： 与 ZI-data 同义。
- .data：与 RW-data 同义

## 5.3 Section Cross References

节区的跨文件引用

```
这部分主要是不同文件中函数的调用关系，详细列出了各个.o 文件之间的符号引用。由于.o 文件是由 asm 或 c/c++源文件编译后生成的，各个文件及文件内的节区间互相独立，链接器根据它们之间的互相引用链接起来，链接的详细信息在这个分区中详细列出。例如：
	***main.o(.text) refers to delay.o(.text) for delay_ms***
```

值得注意的是，在构建工程的时候，有时会出现 “Undefined [symbol](https://so.csdn.net/so/search?q=symbol&spm=1001.2101.3001.7020) xxx (referred from xxx.o)” 这样的错误提示，就是因为链接过程中，某个文件无法从工程所包含的外部找到它引用的标号，因而产生链接错误。

## 5.4 Removing Unused input ps from the image

删除无用节区

```
map 文件的第二部分是删除无用节区的说明，这部分列出了在链接过程它发现工程中未被引用的节区，这些未被引用的节区将会被删除(指不加入到*.axf 文件，不是指在*.o 文件删除)，这样可以防止这些无用数据占用程序空间。
```

虽然我们把 STM32 标准库的各个外设对应的 c 库文件都添加到了工程，但不必担心这会使工程变得臃肿，因为未被引用的节区内容不会被加入到最终的机器码文件中。

- [ ] One ELF Section per Function

编译器在处理一个 c 文件的时候呢，如果这个选项不选，那么这个C文件中的所有函数在编译后只会产生一个叫 .text 的输出节；如果选了呢，那么每个函数将会产生一个输出节，如果你用C写了一个函数，那么编译器将会产生一个叫 相应的输出节。勾选后，编译器产生的输出节，粒度小多了，便于找到没有使用的函数，将它从最后输出文件中删除。这样，你最后产生的可执行文件大小将会变小。
上述选项如果没有勾选，map文件中删除无用节区部分是按照文件优化的，只要该文件中有函数被使用，该文件中所有函数都会被编译进去；勾选后，由于每个函数一个输出节，所以优化是按照函数优化的，没有使用的函数都没有编译进去。

## 5.5 Image Symbol Table 

符号映像表:这个表列出了被引用的各个符号在存储器中的具体地址、占据的空间大小等信息。
分为Local Symbols局部 和 Global Symbols全局。

- **Local Symbols**

  static声明的全局变量地址和大小，C 文件中函数的地址和用 static 声明的 函数代码大小，汇编文件中的标号地址**（作用域限本文件）**

- **Global Symbols**

  全局变量的地址和大小，C 文件中函数的地址及其代码大小，汇编文件中的 标号地址**（作用域全工程）**

1. **Symbol Name：**符号名称

2. **Value**：存储对应的地址；会发现有0x0800xxxx、0x2000xxxx这样的地址。
   0x0800xxxx指存储在FLASH里面的代码、变量等。

   0x2000xxxx指存储在内存RAM中的变量Data等。

3. **Ov Type**：符号对应的类型，大概有几种：Number、Section、Thumb Code、Data等；细心的朋友会发现：全局、静态变量等位于0x2000xxxx的内存RAM中。

4. **Size**：存储大小，这个容易理解，我们怀疑内存溢出，可以查看代码存储大小来分析。

5. **Object(Section)**：当前符号所在段名这里一般指所在模块（所在源文件）。


## 5.6 Memory Map of the image

映像文件可以分为加载域（Load Region）和运行域（Execution Region）：

**加载域（Load Region**）:可执行映像文件的各个段存放在存储器中的位置关系

**运行域（Execution Region）**:可执行映像文件各个段真正执行时在存储器中的位置关系

加载域就是程序在 Flash 中的实际存储,运行域是芯片上电后的运行状态

```
		加载域							运行域
____________________			__________________
--	|							|
	|							|
	|							|
RAM	|							|--------------
	|							|	ZI Section
	|							|--------------
	|							|	RW Section
--  |----------------			|------------------
	|							|
	|							|
	|							|
ROM	|-----------				|
	| RW Section				|
	|-----------				|-----------
	| RO Section				|	RO Section
--	|---------------			|---------------

RW 区也是要存储到 ROM/Flash 里面的，在执行映像之前，必须将已初始化的 RW 数据从 ROM 中复制到 RAM 中的执行地址并创建 ZI Section（初始化为 0 的变量区）。
```

## 5.7 Image component sizes

包含映像组件大小的信息(Image component sizes)，这也是最常查询的内容。
这部分包含了各个使用到的*.o 文件的空间汇总信息、整个工程的空间汇总信息以及占用不同类型存储器的空间汇总信息，它们分类描述了具体占据的 Code、 RO-data、 RW-data及 ZI-data 的大小，并根据这些大小统计出占据的 ROM 总空间。

- Code(inc. Data):显示代码占用了多少字节，inc. Data字节内联数据
- RO Data：显示只读数据占用了多少字节
- RW Data:显示读写数据占用了多少字节
- ZI Data：显示0初始化数据占用了多少字节
- Debug：显示调试数据占用多少字节，调试输入节以及符号和字符串。
- Object Totals：显示链接到一起生成映像的对象占用了多少字节
- (incl. Generated):链接器会生成的映像内容
- (incl. Padding):链接器根据需要插入填充，以强制字节对齐

```c
==============================================================================


      Code (inc. data)   RO Data    RW Data    ZI Data      Debug   

     71532      14216       6344       1680      50480     827927   Grand Totals
     71532      14216       6344        496      50480     827927   ELF Image Totals (compressed)
     71532      14216       6344        496          0          0   ROM Totals

==============================================================================

    Total RO  Size (Code + RO Data)                77876 (  76.05kB)
    Total RW  Size (RW Data + ZI Data)             52160 (  50.94kB)
    Total ROM Size (Code + RO Data + RW Data)      78372 (  76.54kB)

==============================================================================

```

- **Grand Totals：**显示映像的真实大小。
- **ELF Image Totals：**ELF(Executable and Linking Format)可执行链接格式映像文件大小。
- **ROM Totals：**显示包含映像所需的 ROM的最小大小。这不包括 ZI数据和存储在ROM 中的调试信息。

# 6.HTM文件分析

此文件反应所有被调函数的栈stack使用情况，不考虑中断嵌套

```c
Maximum Stack Usage = 884 bytes + Unknown(Cycles, Untraceable Function Pointers)
Call chain for Maximum Stack Depth:
comm_ext_flash_ops ⇒ spi_flash_buffer_write ⇒ spinand_read_dataload ⇒ spinand_hwecc_status_get ⇒ spinand_status_register_read ⇒ nand_spi_write_read ⇒ nand_qspi_send_then_recv ⇒ nand_qspi_transfer_message ⇒ rt_mutex_take ⇒ rt_timer_start ⇒ rt_schedule ⇒ _rt_scheduler_stack_check ⇒ rt_kprintf ⇒ rt_snprintf ⇒ rt_vsnprintf ⇒ print_number
```

# 7.HardFault分析

> apnt209.pdf

![](https://raw.githubusercontent.com/QLiwei/picgo/main/img/entry_interrupt.jpg)



- 进入中断判断LR bit2 ，bit2=0 使用主堆栈指针（MSP)bit2=1使用线程堆栈指针（PSP)
- 进入中断R0,R1,R2,R3,LR,PC,xPSR自动入栈
- 进入中断后 LR,PC,PSR 寄存器的值会发生改变
- 退出中断出栈，寄存器值恢复

```
__BKPT(0); // 断点
```

**MDK 调试：Call Stack Locals 窗口右击 show Caller Code 找到对应的调用者**

**LR 记录进入中断前的最近一次LR寄存器的返回值，下一条指令的地址，PC记录进入中断前下一条指令它的地址Disassembly 窗口 右击 show Disassembly at Address 设置反汇编地址（记录的PC指针） goto 进入中断前下一条指令**



## 7.1 SEGGER

## 7.2 [CMBackTrace](https://github.com/armink/CmBacktrace) 

