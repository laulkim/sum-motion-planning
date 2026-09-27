from __future__ import annotations

from typing import Any

TITLE_FONTSIZE = 20
AXIS_LABEL_FONTSIZE = 17
TICK_LABEL_FONTSIZE = 14
LEGEND_FONTSIZE = 14


def apply_paper_style(axis: Any) -> None:
    """Bold, enlarged title/axis-label/tick/legend text for paper figures.

    Restyles whatever title/labels/legend the caller already set (via
    set_title/set_xlabel/set_ylabel/legend) -- call this last, right before
    savefig.
    """
    title = axis.get_title()
    if title:
        axis.set_title(title, fontsize=TITLE_FONTSIZE, fontweight="bold")
    xlabel = axis.get_xlabel()
    if xlabel:
        axis.set_xlabel(xlabel, fontsize=AXIS_LABEL_FONTSIZE, fontweight="bold")
    ylabel = axis.get_ylabel()
    if ylabel:
        axis.set_ylabel(ylabel, fontsize=AXIS_LABEL_FONTSIZE, fontweight="bold")
    for label in axis.get_xticklabels() + axis.get_yticklabels():
        label.set_fontsize(TICK_LABEL_FONTSIZE)
        label.set_fontweight("bold")
    legend = axis.get_legend()
    if legend is not None:
        for text in legend.get_texts():
            text.set_fontsize(LEGEND_FONTSIZE)
            text.set_fontweight("bold")
