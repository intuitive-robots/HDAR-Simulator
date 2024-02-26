###
{{< doublevideo src1="img/box_stacking.webm" src2="img/cup_inserting.webm">}}
# Abstract
Future versatile robots need the ability to learn new tasks and behaviors from demonstrations. Recent advances in virtual and augmented reality position these technologies as great candidates for the efficient and intuitive collection of large sets of demonstrations. While there are different possible approaches to control a virtual robot there has not yet been an evaluation of these control interfaces in regards to their efficiency and intuitiveness.
These characteristics become particularly important when working with non-expert users and complex manipulation tasks.
To this end, this work investigates five different interfaces to control a virtual robot in a comprehensive user study across various virtualized tasks in an AR setting. These interfaces include Hand Tracking, Virtual Kinesthetic Teaching, Gamepad and Motion Controller. Additionally, this work introduces Kinesthetic Teaching as a novel interface to control virtual robots in AR settings, where the virtual robot mimics the movement of a real robot manipulated by the user.
This study reveals valuable insights into their usability and effectiveness.
It shows that the proposed Kinesthetic Teaching interface significantly outperforms other interfaces in both objective and subjective metrics based on success rate, task completeness, and completion time and User Experience Questionnaires (UEQ+).

<!-- ## Installation
Clone our [Github repository](https://github.com/intuitive-robots/ml-cur) and install with `pip`:
```sh
git clone https://github.com/intuitive-robots/ml-cur.git
pip install ml-cur
```

## Usage
Our public interfaces are inspired by `scikit-learn`. You can also find some Jupyter notebooks in [our demo folder](https://github.com/intuitive-robots/ml-cur/tree/main/demo).

```python {linenos=true}
from ml_cur import MlCurLinMoe
ml_cur_moe = MlCurLinMoe(n_components=2, train_iter=50, num_active_samples=0.4)
ml_cur_moe.fit(train_samples, train_contexts)
``` -->

# Citation
If you find our work useful, please consider citing:

```BibTeX
@inproceedings{jiang2024comprehensive,
  title={A Comprehensive User Study on Augmented Reality-Based Data Collection Interfaces for Robot Learning},
  author={Jiang, Xinkai and Mattes, Paul and Jia, Xiaogang and Schreiber, Nicolas and Neumann, Gerhard and Lioutikov, Rudolf},
  booktitle={Proceedings of the 2024 ACM/IEEE International Conference on Human-Robot Interaction},
  pages={10},
  year={2024},
  organization={ACM},
  address={Boulder, CO, USA},
  date={March 11--14}
}
```