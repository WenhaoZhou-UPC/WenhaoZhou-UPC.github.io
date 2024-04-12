---
tags: [torch]
title: torch系列遇到的一些数据处理方法
created: '2024-04-11T12:40:06.849Z'
modified: '2024-04-11T14:05:40.290Z'
---


### 1、torch.unbind(x)


`torch.unbind(x)`是一个PyTorch张量（Tensor）的函数，它的作用是将张量沿着指定的维度拆分成若干个张量，并返回一个元组（tuple）包含这些张量。

具体来说，`torch.unbind(x)`会将输入张量x沿着指定的维度拆分成若干个张量，例如：

```python
import torch

x = torch.tensor([[1, 2, 3], [4, 5, 6]])
y = torch.unbind(x, dim=1)
print(y)
```


输出结果如下：

```python
(tensor([1, 4]), tensor([2, 5]), tensor([3, 6]))
```


在上面的例子中，我们将输入张量x沿着维度1拆分成了3个张量，这些张量被封装在一个元组中并被返回。在这种情况下，每个输出张量都是沿着列的方向（即水平方向）拆分的。

这个函数在一些深度学习任务中很有用，例如当我们需要对一个张量沿着某个维度进行循环遍历时，可以使用torch.unbind()函数将张量拆分成多个小块，然后对每个小块进行遍历。

### 2、x.new_zeros(x.shape)


用于创建一个形状和数据类型与输入张量x相同，但所有元素都为0的新张量

```python
import torch

x = torch.tensor([[1, 2, 3], [4, 5, 6]])
y = x.new_zeros(x.shape)
print(y)
```


输出结果：

```python
tensor([[0, 0, 0],
        [0, 0, 0]])
```


