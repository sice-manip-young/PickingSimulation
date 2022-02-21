import sys
args = sys.argv
import numpy as np
import plotly.graph_objects as go

data_path = args[1]
depth_image = np.load(data_path)
depth_image = depth_image.reshape(*depth_image.shape[:2])
fig = go.Figure()
fig.update_layout(width = 800, height = 800)
fig.add_trace(go.Surface(z = depth_image))
# fig['layout']['scene']['xaxis_autorange'] = 'reversed'
# fig['layout']['scene']['yaxis_autorange'] = 'reversed'
fig['layout']['scene']['zaxis_autorange'] = 'reversed'
fig['layout']['scene']['zaxis']['range'] = (0,1)
fig.show()
