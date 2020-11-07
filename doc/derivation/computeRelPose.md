
# How to understand `d_rel_d_h`, `d_rel_d_t` in `BundleAdjustmentBase::computeRelPose`.

Thank you for sharing a grate software!!
Basaltのコードを最近調べているのですが、`BundleAdjustmentBase::computeRelPose`内の`d_rel_d_h`, `d_rel_d_t`の計算の理解に手間取っています。自分の理解のどこが間違っているか教えていただけるとありがたいです。
I read code of basalt recently, but I can't understand the calculation of `d_rel_d_h` and `d_rel_d_t` at `BundleAdjustmentBase::computeRelPose`.
Could you please point out the mistakes of my derivation?

`d_rel_d_h`:

`d_rel_d_h`は、`T_i_c_t^T T_w_i_t^T T_w_i_h T_i_c_h`の`T_w_i_h`についてのLeft jacobianと考えています。
よって、


I believe that `d_rel_d_h` is Left jacobian for `T_i_c_t^T T_w_i_t^T T_w_i_h T_i_c_h` on `T_w_i_h`. Therefore, 
$$
\begin{aligned}
\mathrm{d\_rel\_d\_h} &=
\frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh} }{D~ T_{WIh}}\\
&= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh}T_{ICh}}{D~ T_{WIh} T_{ICh}} 
\frac{{}^\mathcal{E} D~ T_{WIh} T_{ICh} }{D~ T_{WIh}}\\
&=\boldsymbol{A}d_{(T_{ICt}^{-1} T_{WIt}^{-1})} \boldsymbol{I}_{66}
\end{aligned}

$$

But, the calculation of `d_rel_d_h` on the code is completely different from my derivation...

しかしながらコード上の`d_rel_d_h`の式は完全に違っています。

`d_rel_d_t`:

I also believe that `d_rel_d_t` is Left jacobian for `T_i_c_t^T T_w_i_t^T T_w_i_h T_i_c_h` on `T_w_i_t`. Therefore, 


`d_rel_d_t`は`T_i_c_t^T T_w_i_t^T T_w_i_h T_i_c_h`の`T_w_i_t`についてのLeft jacobianと考えています。
よって、
$$
\begin{aligned}
\mathrm{d\_rel\_d\_t} &= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh} }{D~ T_{WIt}}\\
&= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh}  }{D~ T_{WIt}^{-1} T_{WIh} T_{ICh}}
\frac{{}^\mathcal{E} D~ T_{WIt}^{-1} T_{WIh} T_{ICh}}{D~ T_{WIt}^{-1}}
\frac{{}^\mathcal{E} D~ T_{WIt}^{-1}}{D~ T_{WIt}}\\
&= \boldsymbol{A}d_{T_{ICt}^{-1}} \boldsymbol{I}_{66} (-\boldsymbol{A}d_{T_{WIt}^{-1}})    
\end{aligned}

$$

This one is a bit close, but it's different from the calculation in the code...

こちらも少し惜しいのですが、コード上の計算と違っています。
間違っているところ、考えを指摘していただけるとありがたいです。


Note:.


$$
\mathrm{Ad}_{M} = \begin{bmatrix}
    R & [\boldsymbol{t}]_\times R\\
    \boldsymbol{0}_{33} & R
\end{bmatrix}
$$

where

$$
M = \begin{bmatrix}
    R & \boldsymbol{t}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix} \in SE(3)
$$

Jacobian $\frac{{{}^{\mathcal{E}}}D f(X)}{D X}$ is taken from equation (44) in http://arxiv.org/abs/1812.01537.



### Post

Thank you for sharing a great software!!

I read code recently, but I can't understand the calculation of `d_rel_d_h` and `d_rel_d_t` in `BundleAdjustmentBase::computeRelPose`.
Could you please point out the mistakes of my derivation?

`d_rel_d_h`:

I believe that `d_rel_d_h` is Left jacobian for `T_i_c_t^-1 T_w_i_t^-1 T_w_i_h T_i_c_h` on `T_w_i_h`. 
Therefore, 
```math
\begin{aligned}
\mathrm{d\_rel\_d\_h} &=
\frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh} }{D~ T_{WIh}}\\
&= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh}T_{ICh}}{D~ T_{WIh} T_{ICh}} 
\frac{{}^\mathcal{E} D~ T_{WIh} T_{ICh} }{D~ T_{WIh}}\\
&=\boldsymbol{A}d_{(T_{ICt}^{-1} T_{WIt}^{-1})} \boldsymbol{I}_{66}
\end{aligned}
```
But, the calculation of `d_rel_d_h` on the code is completely different from my derivation...

`d_rel_d_t`:

I also believe that `d_rel_d_t` is Left jacobian for `T_i_c_t^-1 T_w_i_t^-1 T_w_i_h T_i_c_h` on `T_w_i_t`. 
Therefore, 

```math
\begin{aligned}
\mathrm{d\_rel\_d\_t} &= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh} }{D~ T_{WIt}}\\
&= \frac{{}^\mathcal{E} D~ T_{ICt}^{-1} T_{WIt}^{-1} T_{WIh} T_{ICh}  }{D~ T_{WIt}^{-1} T_{WIh} T_{ICh}}
\frac{{}^\mathcal{E} D~ T_{WIt}^{-1} T_{WIh} T_{ICh}}{D~ T_{WIt}^{-1}}
\frac{{}^\mathcal{E} D~ T_{WIt}^{-1}}{D~ T_{WIt}}\\
&= \boldsymbol{A}d_{T_{ICt}^{-1}} \boldsymbol{I}_{66} (-\boldsymbol{A}d_{T_{WIt}^{-1}})    
\end{aligned}
```
This one is a bit close, but it's different from the calculation in the code...

Note:

- Adjoint $`M \in SE(3)`$ is
```math
\mathrm{Ad}_{M} = \begin{bmatrix}
    R & [\boldsymbol{t}]_\times R\\
    \boldsymbol{0}_{33} & R
\end{bmatrix}
```

where

```math
M = \begin{bmatrix}
    R & \boldsymbol{t}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix}.
```

- Jacobian $`\frac{{{}^{\mathcal{E}}}D f(X)}{D X}`$ is taken from equation (44) in http://arxiv.org/abs/1812.01537.



# Reply
## Nikolaus
- 導出は見た感じOK
- しかしBasaltでは decoupled left incrementを使っているので、Chain ruleをもう一回かける必要がある。
- Decouled left incrementについてのでFull SE3のincrementの微分はこれ
- tは、微分しようとしているSE3のTranslation
- おれの結果にあとからこれを掛けると、Code上と同じ数式になる。
- 数値微分でも、この結果は検証可能。TestコードをFull SE3のIncrementに変更しないと、Testに失敗すると思うよ。


## Kazuki
- 自分の導出に、Decoupledへの変換行列をかけたら、Codeの数式を得られた。
- また、TestをFull SE3 left incrementに変更して、Full SE3のDerivationを数値微分で検証すると、Passすることもわかった。
- 一方、 Decoupled left incrementや、変換行列の導出についてはよく理解できていない。以下の解釈でOK？
- 俺の解釈：
Decoupled left incrementは変換行列を掛けることで、Full SE3 left incrementと同じ結果をえることができている。
数式で書くと以下のようなイメージ。

関数$f(X) \in SE3, X \in SE3$であるとする。
また、
$\oplus$ : SE3 full left increment, 
$\boxplus$ : Decoupled left increment
とする。

このとき、$\boldsymbol{\tau} = [\boldsymbol{v}, \boldsymbol{\omega}]^T \rightarrow \boldsymbol{0}$であるとき、
$$
\begin{aligned}
f(\tau \oplus X) \rightarrow &\frac{{}^{\mathcal{E}}D f(X)}{D X} \boldsymbol{\tau} \oplus X \\
&= \frac{{}^{\mathcal{E}}D f(X)}{D X}
\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times \\
    \boldsymbol{0}_{33} & I_3
\end{bmatrix}
 \boldsymbol{\tau} \boxplus X
\end{aligned}

$$
として、Full SE3 incrementの代わりにDecoupled left incrementを使ってSE3のIncrementを計算していると理解した。



- Full SE3 left incrementに対して、計算量的には有利なのは理解できる。
- もし、Decoupled incrementや、他の<R^3, SO3>, [R^3 x SO3]について参考になる文献があれば教えてほしい。


## Post

I'm sorry for late reply.
Thank you so much for your kindly answer!!

I could successfully get the expression in the code by post-multiplying the matrix to my derivations.
And also, I conformed that the jacobian test with my derivations passes when replacing decoupled left increment to full SE3 left increment!

However, decoupled left increment and the conversion matrix 
$\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times \\
    \boldsymbol{0}_{33} & I_3
\end{bmatrix}$
are still unclear to me.
Decoupled left increment with conversion matrix dose the same calculation as Full SE3 left increment? like this,

$$
\begin{aligned}
f(\tau \oplus X) \rightarrow &\frac{{}^{\mathcal{E}}D f(X)}{D X} \boldsymbol{\tau} \oplus X \\
&= \frac{{}^{\mathcal{E}}D f(X)}{D X}
\begin{bmatrix}
    I_3 & [\boldsymbol{t}]_\times \\
    \boldsymbol{0}_{33} & I_3
\end{bmatrix}
 \boldsymbol{\tau} \boxplus X,
\end{aligned}
$$
where,
$$
\boldsymbol{\tau} = \begin{bmatrix}
    \boldsymbol{v}\\
    \boldsymbol{\omega}
\end{bmatrix} \rightarrow \boldsymbol{0},
$$
$$
f: SE3 \rightarrow SE3,\\
X = \begin{bmatrix}
    R & \boldsymbol{t}\\
    \boldsymbol{0}_{13} & 1
\end{bmatrix},
$$

$\oplus$ : Full SE3 left increment,

$\boxplus$ : Decoupled left increment,

$\frac{{}^{\mathcal{E}}D f(X)}{D X}$ : Full SE3 Left jacobian of f(X) w.r.t. X.




I will appreciate you if you could recommend me some references about decoupled left increment.

# Reply 2

## Nikolaus Demmel


## KK