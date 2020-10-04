# linearizePoint

## res
ベースとなるReprojection errorの式。論文と右辺の式のマイナスする順序が逆になっているので注意。
$$
\mathrm{res} = \boldsymbol{r}_{it} = 
\pi ( T_{W C_t}^{-1} T_{W C_h} \boldsymbol{q}({}_h \boldsymbol{m}_i)) - \boldsymbol{z}_{it}\\
= \pi (T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i))
$$

このとき、
- $\pi$ : Projection関数
- $\boldsymbol{q}(u,v,d) = [x,y,z,d^{-1}]^T = [\boldsymbol{p}, d^{-1}]^T$ : Stereographic projection関数
- ${}_h \boldsymbol{m} = [u,v,d]^T$ : Minimal representation.平面場位置$u,v$とその点までの距離d(HostFrame座標系)によってランドマーク位置を表現している。

また、
$$
T_{th} = 
\begin{bmatrix}
R_{th} & \boldsymbol{t}_{th}\\
\boldsymbol{0}_{13} & 1
\end{bmatrix} \\
\boldsymbol{q} = 
\begin{bmatrix}
\boldsymbol{p}\\
d^{-1}
\end{bmatrix}
$$


## d_res_d_xi

再投影誤差の $T_{th}$ についてのJacobian。
$\mathcal{E}$ における接空間(Lie algebra)で表現しているので注意。
$$ \frac{{}^{\mathcal{E}}D \boldsymbol{r}_{it}}{D T_{th}} = 
J_p
\begin{bmatrix}
d^{-1} I_{33} & - [R_{th} \boldsymbol{p} + d^{-1} \boldsymbol{t}_{th}]_{\times} \\
\boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}
$$

### 導出
Reprojection errorの式から、
$$
\frac{{}^{\mathcal{E}}D \boldsymbol{r}_{it}}{D T_{th}} 
= 
\frac{\partial \pi}{\partial (T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i))}
\frac{{}^{\mathcal{E}}D T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i)}{D T_{th}}
= 
\frac{\partial \pi}{\partial (T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i))}
\boldsymbol{Ad}_{T_{th}\boldsymbol{q}({}_h\boldsymbol{m}_i)}
\frac{{}^{\mathcal{T_{th}}}D T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i)}{D T_{th}}
\boldsymbol{Ad}_{T_{th}^{-1}}
$$
が成り立つ。個々の要素を求めていくことで`d_res_d_xi`が計算できる。

まず、Projection関数のJacobianを$J_p$とおく。これはProjection関数（カメラモデル）が変わると内容が変化する。
$$
\frac{\partial \pi}{\partial (T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i))} = J_p
$$

次に、SE3($T_{th}$)と$\boldsymbol{q}$間のaction($T_{th}\boldsymbol{q}$)についてJacobianを計算する。ここでは、$T_{th}$の接空間におけるJacobianを求め、その後$\mathcal{E}$の接空間におけるJacobianに変換する。
まず、$T_{th}$の接空間におけるJacobianは、
$$
\frac{{}^{\mathcal{T_{th}}}D T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i)}{D T_{th}}=
\begin{bmatrix}
    d^{-1}R_{th} & -R_{th}[\boldsymbol{p}]_\times\\
    \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}
$$
となる。$\boldsymbol{q}$の第４要素はInverse distance($d^{-1}$)なので、通常のSE3におけるGroup actionとは異なっていることに注意。
変換するためのAdjointは以下の通り。
$$
\boldsymbol{Ad}_{T_{th}\boldsymbol{q}({}_h\boldsymbol{m}_i)} = I_{33} \\
\boldsymbol{Ad}_{T_{th}^{-1}} =
\begin{bmatrix}
    R_{th}^T & [-R_{th}^T \boldsymbol{t}_{th}]_\times R_{th}^T\\
    \boldsymbol{0}_{33} & R_{th}^T
\end{bmatrix}
$$

よって、$\mathcal{E}$の接空間におけるJacobianは以下の通り。
$$
\frac{{}^{\mathcal{E}}D T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i)}{D T_{th}}=
\boldsymbol{Ad}_{T_{th}\boldsymbol{q}({}_h\boldsymbol{m}_i)}
\frac{{}^{\mathcal{T_{th}}}D T_{th} \boldsymbol{q}({}_h \boldsymbol{m}_i)}{D T_{th}}
\boldsymbol{Ad}_{T_{th}^{-1}}
=
I_{33}
\begin{bmatrix}
    d^{-1}R_{th} & -R_{th}[\boldsymbol{p}]_\times\\
    \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}
\begin{bmatrix}
    R_{th}^T & [-R_{th}^T \boldsymbol{t}_{th}]_\times R_{th}^T\\
    \boldsymbol{0}_{33} & R_{th}^T
\end{bmatrix}\\
=
\begin{bmatrix}
    d^{-1} R_{th} R_{th}^T & d^{-1} R_{th}[-R_{th}^{T} \boldsymbol{t}_{th}]_\times R_{th}^T - R_{th}[\boldsymbol{p}]_\times R_{th}^T \\
    \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}\\
=
\begin{bmatrix}
    d^{-1} I_{33} & - R_{th}[d^{-1} R_{th}^T \boldsymbol{t}_{th} + \boldsymbol{p} ]_\times R_{th}^T \\
    \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}\\
=
\begin{bmatrix}
    d^{-1} I_{33} & - [d^{-1} \boldsymbol{t}_{th} + R_{th} \boldsymbol{p} ]_\times \\
    \boldsymbol{0}_{13} & \boldsymbol{0}_{13}
\end{bmatrix}
$$