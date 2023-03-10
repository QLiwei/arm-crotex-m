# Cortex-M3权威指南

> Joseph Yin 著
>
> 宋岩 译
>
> 2023/2/7 开始学习

## chpt01 介绍

CM3的招牌功夫包括：

• 性能强劲。在相同的主频下能做处理更多的任务，全力支持劲爆的程序设计。

• 功耗低。延长了电池的寿命——这简直就是便携式设备的命门（如无线网络应用）。

• 实时性好。采用了很前卫甚至革命性的设计理念，使它能极速地响应中断，而且响应中断所

需的周期数是确定的。

• 代码密度得到很大改善。一方面力挺大型应用程序，另一方面为低成本设计而省吃俭用。

• 使用更方便。现在从8位/16位处理器转到32位处理器之风刮得越来越猛，更简单的编程模型

和更透彻的调试系统，为与时俱进的人们大大减负。

• 低成本的整体解决方案。让32位系统比和8位/16位的还便宜，低端的Cortex-M3单片机甚至还

卖不到1美元。

• 遍地开花的优秀开发工具。免费的，便宜的，全能的，要什么有什么。

## chpt04 指令集

### 4.1 汇编语言基础

#### 4.1.1 基础语法

```
标号
	操作码		操作数1,	操作数2, 	...	;注释
```

- 标号是可选的，如果有，它必须顶格写,标号的作用是让汇编器来计算程序转移的地址
- 操作码是指令的助记符，它的前面必须有至少一个空白符
- 操作码后面往往跟随若干个操作数，而第 1 个操作数，通常都给出本指令的执行结果存储处
- 立即数必须以“#”开头
- 注释均以”;”开头

如果汇编器不能识别某些特殊指令的助记符,需要进行"手工汇编"-查出该指令的确切二进制机器码,使用DCI编译器指示字.

例如, BKPT指令机器码为0xBE00

```c
	DCI		0xBE00	;断点(BKPT) 16位指令
```

[注]:DCI前面需留出空白符

使用DCB定义一串字节常数

DCD定义一串32位整数

```c
	LDR R3,	=MY_NUMBER	;	R3= MY_NUMBER
   	LDR	R4,	[R3]		;	R4= *R3
        ...
    LDR	R0,	=HELLO_TEXT	;	R0= HELLO_TEXT
    BL	PrintText		;	call PrintText以显示字符串,R0传递参数
        ...
MY_NUMBER
   	DCD	0x12345678
HELLO_TEXT
    DCB	"Hello\n",0
```

#### **4.1.2 后缀的使用**

| 后缀名                 | 含义                                                         |
| ---------------------- | ------------------------------------------------------------ |
| S                      | 要求更新 APSR 中的相关标志，<br />例如：ADDS R0, R1 ; 根据加法的结果更新 APSR 中的标志 |
| **EQ,NE,LT,GT** **等** | 有条件地执行指令。EQ=Euqal, NE= Not Equal, LT= Less Than, GT= Greater Than<br />BEQ <Label> ; 仅当 EQ 满足时转移 |

在 Cortex-M3 中，对条件后缀的使用有很大的限制：只有转移指令（B 指令）才可随意使用。而对于其它指令，CM3 引入了 IF-THEN 指令块，在这个块中才可以加后缀，且必须加以后缀

#### 4.1.3 **统一汇编语言书写语法**

为了最有力地支持 Thumb-2，也作为对汇编程序员的人文关怀，ARM 汇编器引了一个“**统一汇编语言（UAL）**”语法机制.对于 16 位指令和 32 位指令均能实现的一些操作（常见于数据处理操作），有时虽然指令的实际操作数不同，或者对立即数的长度有不同的限制，但是汇编器允许开发者统一使用 32 位 Thumb-2 指令的语法格式书写（很多 Thumb-2 指令的用法也与 32 位 ARM 指令相同），并且由汇编器来决定是使用 16 位指令，还是使用 32 位指令。

Thumb 的语法和 ARM 的语法不同，在有了 UAL 之后，两者的书写格式就统一了。

```
	ADD	R0,	R1		;	传统的Thumb语法
	ADD	R0,	R0,	R1	;	引入UAL后允许的等效写法 R0=R0+R1
```

然引入了 UAL，但是仍然允许使用传统的 Thumb 语法。不过有一项必须注意：如果使用传统的 Thumb 语法，有些指令会默认地更新 APSR，即使你没有加上 S 后缀。如果使用 UAL 语法，则必须指定 S 后缀才会更新。例如：

```c
	ADD		R0,	R1		;	传统的 Thumb 语法
	ADDS	R0,	R0,	R1	;	等值的 UAL 语法（必须有 S 后缀）
```

在 Thumb-2 指令集中，有些操作既可以由 16 位指令完成，也可以由 32 位指令完成。例如，R0=R0+1 这样的操作，16 位的与 32 位的指令都提供了助记符为“ADD”的指令。在 UAL 下，汇编器能主动决定用哪个，也可以手工指定是用 16 位的还是 32 位的：

```c
	ADDS	R0,	#1		;	汇编器将为了节省空间而使用 16 位指令
	ADDS.N	R0,	#1		;	指定使用 16 位指令（N＝Narrow）
	ADDS.W	R0.	#1		;	指定使用 32 位指令（W=Wide）
```

.W(Wide)后缀指定 32 位指令。如果没有给出后缀，汇编器会先试着用 16 位指令以给代码瘦身，如果不行再使用 32 位指令。

绝大多数16位指令只能访问R0-R7；32位Thumb-2指令则可以随意访问R0-R15。不过 ，把R15(PC)作为目的寄存器很容易走火入魔——用对了会有意想不到的妙处，出错时则会使程序跑飞。通常只有系统软件才会不惜冒险地做此高危行为，因此还需慎用。对 PC 的使用还有其它的戒律，如果感兴趣，可以参考《ARMv7-M 架构应用级参考手册》。



在讲指令之前，先简单地介绍一下 Cortex-M3 中支持的算术与逻辑标志。

APSR 中的 5 个标志位

- N: 负数标志(Negative)

- Z： 零结果标志(Zero)

- C: 进位/借位标志(Carry)

- V: 溢出标志(oVerflow)

- S: 饱和标志(Saturation)，它不做条件转移的依据



### 4.3 指令使用

#### 4.3.1 数据传送

处理器的基本功能之一就是数据传送。CM3 中的数据传送类型包括

- 在两个寄存器间传送数据

- 在寄存器与存储器间传送数据

- 在寄存器与特殊功能寄存器间传送数据

- 把一个立即数加载到寄存器

**寄存器间传送数据**

```c
	MOV	R8,	R3	;	R3的数据传送到R8
```

MOV 的一个衍生物是 MVN，它把寄存器的内容取反后再传送

**用于访问存储器的基础指令是“加载（Load）”和“存储（Store）”**

常用的存储器访问指令:

| 示例                             | 功能描述                                                     |
| -------------------------------- | ------------------------------------------------------------ |
| **LDRB   Rd,  [Rn, #offset]**    | 从地址 Rn+offset 处读取一个字节送到 Rd                       |
| **LDRH Rd, [Rn, #offset]**       | 从地址 Rn+offset 处读取一个半字送到 Rd                       |
| **LDR Rd, [Rn, #offset]**        | 从地址 Rn+offset 处读取一个字送到 Rd                         |
| **LDRD Rd1, Rd2, [Rn, #offset]** | 从地址 Rn+offset 处读取一个双字(64 位整数)送到 Rd1（低 32 位）和 Rd2（高 32 位）中 |
| **STRB Rd, [Rn, #offset]**       | 把 Rd 中的低字节存储到地址 Rn+offset 处                      |
| **STRH Rd, [Rn, #offset]**       | 把 Rd 中的低半字存储到地址 Rn+offset 处                      |
| **STR Rd, [Rn, #offset]**        | 把 Rd 中的低字存储到地址 Rn+offset 处                        |
| **STRD Rd1, Rd2, [Rn, #offset]** | 把 Rd1（低 32 位）和 Rd2（高 32 位）表达的双字存储           |

常用的多重存储器访问方式:

| 示例                          | 功能描述                                                     |
| ----------------------------- | ------------------------------------------------------------ |
| **LDMIA Rd!, {寄存器列表}**   | 从 Rd 处读取多个字，并依次送到寄存器列表中的寄存器。每读一个字后 Rd 自增一次，16 位宽度 |
| **STMIA Rd!, {寄存器列表}**   | 依次存储寄存器列表中各寄存器的值到 Rd 给出的地址。每存一个字后 Rd 自增一次，16 位宽度 |
| **LDMIA.W Rd!, {寄存器列表}** | 从 Rd 处读取多个字，并依次送到寄存器列表中的寄存器。每读一个字后 Rd 自增一次，32 位宽度 |
| **LDMDB.W Rd!, {寄存器列表}** | 从 Rd 处读取多个字，并依次送到寄存器列表中的寄存器。每读一个字前 Rd 自减一次，32 位宽度 |
| **STMIA.W Rd!, {寄存器列表}** | 依次存储寄存器列表中各寄存器的值到 Rd 给出的地址。每存一个字后 Rd 自增一次，32 位宽度 |
| **STMDB.W Rd!, {寄存器列表}** | 存储多个字到 Rd 处。每存一个字前 Rd 自减一次，32位宽度       |

上表中，加粗的是符合 CM3 堆栈操作的 LDM/STM 使用方式。并且，如果 Rd 是 R13（即 SP），则与 POP/PUSH指令等效。(LDMIA->POP, STMDB -> PUSH)

```
STMDB SP!, {R0-R3, LR} ;等效于 PUSH {R0-R3, LR}
LDMIA SP!, {R0-R3, PC} ;等效于 POP {R0-R3, PC}
```

Rd后面的“！”是什么意思？它表示要自增(Increment)或自减（Decrement）基址寄存器Rd的值，时机是在每次访问前(Before)或访问后(After)。增/减单位：字（4 字节）

#### 4.3.2 数据处理

```
	ADD	R0,	R1	;R0 += R1
	ADD	R0,	#0x12	;R0 += 0x12
	ADD.W R0,	R1,	R2	;R0 = R1 + R2
```

含 SUB, MUL, UDIV/SDIV 等用于算术四则运算

#### 4.3.3 子程呼叫与无条件跳转指令

最基本的无条件跳转指令有两条：

```
	B	Lable	;	跳转到Lable处对应的地址
	BX	reg		;	跳转到由寄存器reg给出的地址
```

在 BX 中，reg 的最低位指示出在转移后将进入的状态：是 ARM(LSB=0)呢，还是Thumb(LSB=1)。

呼叫子程序时，需要保存返回地址，正点的指令是：

```c
	BL	Lable	;	跳转到 Label 对应的地址，并且把跳转前的下条指令地址保存到 LR
    BLX	reg		;	跳转到由寄存器 reg 给出的地址，并根据 REG 的 LSB 切换处理器状态
        		;	还要把转移前的下条指令地址保存到 LR
```

#### 4.3.4 **标志位与条件转移**

| 标志位 | PSR位序号 | 功能描述                                                     |
| ------ | --------- | ------------------------------------------------------------ |
| N      | 31        | 负数（上一次操作的结果是个负数）。N=操作结果的 MSB           |
| Z      | 30        | 零（上次操作的结果是 0）。当数据操作指令的结果为 0,或者比较/测试的结果为 0 时，Z 置位。 |
| C      | 29        | 进位／借位（上次操作导致了进位或者借位）。C 用于无符号数据处理，最常见的就是当加法进位及减法借位时 C 被置位。此外，C还充当移位指令的中介（详见 v7M 参考手册的指令介绍节）。 |
| V      | 28        | 溢出（上次操作结果导致了数据的溢出）。该标志用于带符号的数据处理。比如，在两个正数上执行 ADD 运算后，和的 MSB 为 1（视作负数），则 V 置位。 |

