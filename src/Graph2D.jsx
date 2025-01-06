import React from 'react';
import {ForceGraph2D} from 'react-force-graph';

export default function Graph2D(props) {
  const {graphData, handleNodeClick, handleLeftClick} = props;
  const fgRef = React.useRef(null);
  return (
    <ForceGraph2D
      ref={fgRef}
      graphData={graphData}
      backgroundColor="#000003"
      linkColor={()=>'rgba(255,255,255,0.1)'}
      onNodeClick={handleLeftClick}
      onNodeRightClick={handleNodeClick}
      linkDirectionalArrowLength={0}
      linkDirectionalArrowRelPos={1}  
      linkDirectionalParticles={1}
      onEngineStop={() => console.log('engine stops')}
      onNodeDragEnd={node => {
        node.fx = node.x;
        node.fy = node.y;
        node.fz = node.z;
      }}
      nodeCanvasObject={(node, ctx, globalScale) => {
        const label = node.text;
        const fontSize = 12/globalScale;
        ctx.font = `${fontSize}px Sans-Serif`;
        const textWidth = ctx.measureText(label).width;
        const bckgDimensions = [textWidth, fontSize].map(n => n + fontSize * 0.2); // some padding

        ctx.fillStyle = 'rgba(0, 0, 0, 0.8)';
        ctx.fillRect(node.x - bckgDimensions[0] / 2, node.y - bckgDimensions[1] / 2, ...bckgDimensions);

        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = node.color;
        ctx.fillText(label, node.x, node.y);

        node.__bckgDimensions = bckgDimensions; // to re-use in nodePointerAreaPaint
      }}
      nodePointerAreaPaint={(node, color, ctx) => {
        ctx.fillStyle = color;
        const bckgDimensions = node.__bckgDimensions;
        bckgDimensions && ctx.fillRect(node.x - bckgDimensions[0] / 2, node.y - bckgDimensions[1] / 2, ...bckgDimensions);
      }}
    >
    </ForceGraph2D>
  )
}
