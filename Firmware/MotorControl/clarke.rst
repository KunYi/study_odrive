Clarke 變換
==============

**1. 為何需要 Clarke 變換？**

在三相平衡交流系統中，通常會有三個相電壓/電流（Va, Vb, Vc），它們是彼此相差 120° 的正弦波。然而，這三個變數並非完全獨立，它們本身也受 **克希荷夫電流定律 (KCL)** 限制。把負載看成單一節點使用，所以一定會滿足 **KCL**：

.. math::
   V_a + V_b + V_c = 0

由於這個限制，三個變數中只有兩個是獨立的。**Clarke 變換** 的目的就是找到一個 **變換矩陣 T**，將三相電壓/電流 (Va, Vb, Vc) 轉換到一個二維的正交 **α-β 靜止坐標系** (Vα, Vβ) 中：

.. math::
   \left[
   \begin{matrix}
    V_\alpha \\
    V_\beta
   \end{matrix}
   \right]
   = T *
   \left[
   \begin{matrix}
   V_a \\
   V_b \\
   V_c
   \end{matrix}
   \right]

這樣做的優點是，我們可以使用兩個獨立的變數 :math:`\left(\textbf{V$_\mathbf{\alpha}, V_\mathbf{\beta}$}\right)` 在二維平面上 **完整地描述** 原始三相系統的電壓/電流資訊，簡化後續的分析和控制。

**2. 幾何直覺**

為了更好地理解 Clarke 變換，我們可以從幾何的角度來看：

-   **原始三相坐標 (Va, Vb, Vc)**：每一相電壓可以想像成對應到一個基底向量，這些向量之間相隔 120°。
-   **目標 α-β 二維坐標系**：我們希望建立一個 **正交基底** 來表示新的二維系統，方便分析：

    -   **α 軸**：方向與 A 相（Va）相同。
    -   **β 軸**：方向與 α 軸正交。

.. image:: ../../study/ClarkeCoordinateSystem_01.png


Clarke 變換的幾何意義就是將原始三相系統中的每個相電壓 **投影** 到這個新的正交 α 和 β 軸上，得到它們在二維平面上的分量 Vα 和 Vβ。

**3. 基底向量推導**

根據上述幾何直覺，我們可以推導出 Clarke 變換的基底向量和轉換矩陣：

-   **Va 的基底表示**: :math:`\left(\begin{matrix} 1_, 0\end{matrix}\right)`
-   **Vb 的基底表示**: :math:`\left(\begin{matrix} -\frac{1}{2}_, \frac{\sqrt{3}}{2}\end{matrix}\right)`
-   **Vc 的基底表示**: :math:`\left(\begin{matrix} -\frac{1}{2}_, -\frac{\sqrt{3}}{2}\end{matrix}\right)`

基於這些基底向量，我們可以 **構造出 Clarke 變換的矩陣 T**：

.. math::

   T =
   \left[
   \begin{matrix}
   1 & -\frac{1}{2} & -\frac{1}{2} \\
   0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2}
   \end{matrix}
   \right]


**4. 矩陣驗證與標準公式**

將三相電壓 [Va, Vb, Vc]ᵀ 帶入上述轉換矩陣，我們可以得到 α 和 β 分量：

.. math::
       V_\alpha &= V_a - \frac{1}{2}V_b - \frac{1}{2}V_c \\
       V_\beta &= \frac{\sqrt{3}}{2}V_b - \frac{\sqrt{3}}{2}V_c

當信號為電壓或是電流時，我們還需要乘上　:math:`k = \frac{2}{3}` 才會讓信號維持相同振幅。而套用 **KCL**　原則，則可以進一步簡化為下列形式

.. math::
      V_a &= -(V_b + V_c) \\
      V_\alpha &= k(V_a - \frac{1}{2}(V_b + V_c)) \\
      &= \frac{2}{3}(V_a + \frac{1}{2}V_a) \\
      &= V_a \\
      \\
      V_\beta &= k\frac{\sqrt{3}}{2}(V_b - V_c) \\
      &= \frac{2}{3}\frac{\sqrt{3}}{2}(V_b - V_c) \\
      &=\frac{\sqrt{3}}{3}(V_b - V_c)\quad\dots\text{將分子, 分母同乘}\sqrt{3} \\
      &=\frac{1}{\sqrt{3}}(V_b - V_c)



倘若系統不平衡的話，需要另外一個零序分量，我們現在只考慮平衡狀態

**5. Clarke 變換的程式碼實現**

在程式碼中，Clarke 變換通常會接收三相電流（Ia, Ib, Ic）或電壓作為輸入，並輸出兩相的 α-β 分量（Iα, Iβ 或 Vα, Vβ）。根據提供的程式碼片段，以下是一個 Clarke 變換的實現範例（針對電流）：

.. code-block:: c++

   std::optional<float2D> Ialpha_beta;

   if (currents.has_value()) {
       // Clarke transform
       Ialpha_beta = {
           (*currents),
           one_by_sqrt3 * ((*currents) - (*currents))
       };
   }

以及另一個範例：

.. code-block:: c++

   float I_alpha_beta = {
       current_meas->phA,
       one_by_sqrt3 * (current_meas->phB - current_meas->phC)};

在這些程式碼片段中，**三相電流被轉換為兩相靜止坐標系下的電流分量 Iα 和 Iβ**。``one_by_sqrt3`` 代表 **1/√3**。需要注意的是，Clarke 變換存在不同的版本，它們在最終的 α-β 分量上可能會有一個比例因子，例如 **功率保持不變** 的版本和 **幅值保持不變** 的版本。在實際應用中，應根據具體需求選擇合適的版本。

總結來說，**Clarke 變換** 是一種重要的坐標轉換，它將三相交流系統的變數轉換到一個更易於分析和控制的二維靜止坐標系中，是 **場導向控制 (FOC)** 等現代電機控制技術的基礎。
