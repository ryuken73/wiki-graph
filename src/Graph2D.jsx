import React from 'react';
import {ForceGraph2D} from 'react-force-graph';

function Graph2D(props, graphRef) {
  const {graphData, handleNodeClick, handleLeftClick} = props;
  const [hightlightNodes, setHighligntNodes] = React.useState(new Set());
  // const fgRef = React.useRef(null);

  const updateHighlight = React.useCallback(() => {
    setHighligntNodes(hightlightNodes)
  }, [])

  const handleNodeHover = React.useCallback((node) => {
    setHighligntNodes((hightlightNodes) => {
      hightlightNodes.clear();
      if(node){
        console.log(node, node.neighbors)
        hightlightNodes.add(node);
        node.neighbors.forEach(neighbor => hightlightNodes.add(neighbor));
      }
      return hightlightNodes
    })
    // hightlightNodes.clear();
    // if(node){
    //   hightlightNodes.add(node);
    //   node.neighbors.forEach(neighbor => hightlightNodes.add(neighbor));
    //   updateHighlight();
    // }
  }, [])
  return (
    <ForceGraph2D
      ref={graphRef}
      graphData={graphData}
      // backgroundColor="#000003"
      backgroundColor="transparent"
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
      onNodeHover={handleNodeHover}
      nodeCanvasObject={(node, ctx, globalScale) => {
        const label = node.text;
        const fontSize = 12/globalScale;
        ctx.font = `${fontSize}px Sans-Serif`;
        const textWidth = ctx.measureText(label).width;
        const bckgDimensions = [textWidth, fontSize].map(n => n + fontSize * 0.5); // some padding

        ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
        ctx.fillRect(node.x - bckgDimensions[0] / 2, node.y - bckgDimensions[1] / 2, ...bckgDimensions);

        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = node.color;
        ctx.fillText(label, node.x, node.y);

        const needHighlight = hightlightNodes.has(node);
        if(needHighlight){
          ctx.lineWidth = 0.5;
          ctx.beginPath();
          ctx.strokeRect(node.x - bckgDimensions[0] / 2, node.y - bckgDimensions[1] / 2, ...bckgDimensions);
          ctx.strokeStyle = 'yellow';
        }

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

export default React.memo(React.forwardRef(Graph2D));