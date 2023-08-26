1.根据CAD的距离角度以及图形文件在JOSM上进行绘制。
（如果想要显示图片作为背景进行描图的话，使用PicLayer插件，请保存calibration文件.cal）

2.为每个area和passage分配osm的基本tag，具体参照osm_standard文件。

3.注意！！！！！由于使用JOSM绘制和修改会改变Node和Way的ID（因为我们的action是“modify”，id为负数），而我们的passage中的from和to的搜索目前是利用id进行搜索的！！！
因此，在为每个passage分配语义标签时，请妥善保存该文件到semantic_passage文件夹，以便于输入到之后的（语义tag -> id）修改脚本中!!!!

5.data_parser库对原始的osm文件有着一些严苛而不合理的要求（后续也许会进行修改）: (1) 读取时要先读取area再读取passage，所以在area的信息需要放在passage前（暂时手动调整）。
（2）我们需要对每个osm进行rootnode的设置，作为坐标原点。

6.最后的osm文件请参考fix_id中的格式进行修改，修改后的文件请勿在JOSM中进行修改！！！如果要修改请修改对应的带有语义标签的文件，并重新替换为id。
