import matplotlib.pyplot as plt

def get_label(ax: plt.Axes, label: str = '_nolegend_') -> str:
    _, labels = ax.get_legend_handles_labels()
    return label if label not in labels else '_nolegend_'
