import React from 'react';
import {ForceGraph2D} from 'react-force-graph';

function Graph2D(props, graphRef) {
  const {graphData, handleNodeClick, handleLeftClick} = props;
  const [highlightNodes, setHighligntNodes] = React.useState(new Set());
  const [highlightLinks, setHighligntLinks] = React.useState(new Set());
  // const fgRef = React.useRef(null);

  const updateHighlight = React.useCallback(() => {
    setHighligntNodes(highlightNodes)
  }, [])

  const handleNodeHover = React.useCallback((node) => {
    setHighligntNodes((highlightNodes) => {
      highlightNodes.clear();
      if(node){
        console.log(node, node.neighbors)
        highlightNodes.add(node);
        node.neighbors?.forEach(neighbor => highlightNodes.add(neighbor));
      }
      return highlightNodes
    })
    setHighligntLinks((highlightLinks) => {
      highlightLinks.clear();
      if(node){
        console.log(node, node.links)
        node.links?.forEach(link => highlightLinks.add(link));
      }
      return highlightLinks
    })
    // highlightNodes.clear();
    // if(node){
    //   highlightNodes.add(node);
    //   node.neighbors.forEach(neighbor => highlightNodes.add(neighbor));
    //   updateHighlight();
    // }
  }, [])
  return (
    <ForceGraph2D
      ref={graphRef}
      graphData={graphData}
      // backgroundColor="#000003"
      backgroundColor="transparent"
      linkColor={(link)=> highlightLinks.has(link)? 'rgba(234, 239, 44,0.5)': 'rgba(255,255,255,0.2)'}
      linkWidth={(link)=> highlightLinks.has(link) ? 5: 1}
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

        const needHighlight = highlightNodes.has(node);
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