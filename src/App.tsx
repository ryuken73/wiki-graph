import React from 'react';
import styled from 'styled-components';
import {ForceGraph2D} from 'react-force-graph';
import {getBacklinksByContentId} from './js/serverApi.js';
import {mkNetworkData} from './js/dataHandlers.js';
import './App.css'

const Container = styled.div``
const RefreshButton = styled.button`
  position:  absolute;
  top: 0;
  right: 0;
  z-index: 10;  
`

const COLORS = {
  person: 'yellow',
  other: 'grey'
}
const contentId = '가수_한국_C_004301_유재석';
const initialNode = {
  id:  contentId,
  text: '유재석',
  color: COLORS.person,
  value: 100
}

function App() {
  const [lastNetworkData, setLastNetworkData] = React.useState({nodes:[initialNode], links:[]});
  const fgRef = React.useRef(null);
  React.useEffect( () => {
    getBacklinksByContentId(contentId)
    .then((result: unknown) => {
      console.log(result)
      setLastNetworkData((lastNetworkData) => {
        const newNetworkData = mkNetworkData(result, contentId, lastNetworkData);
        return newNetworkData;
      })
    })
  }, [])
  const handleNodeClick = React.useCallback(async (node)  => {
    const {id, isPerson} = node;
    if(!isPerson){
      return false
    }
    console.log(id, isPerson)
    const rows = await getBacklinksByContentId(id)
    const newNetworkData = mkNetworkData(rows, node.id, lastNetworkData);
    setLastNetworkData(newNetworkData)
  }, [lastNetworkData]);
  const refreshGraph = React.useCallback(() => {
    console.log(fgRef.current)
    // fgRef.current.refresh()
  }, [])

  return (
    <Container>
      <RefreshButton onClick={refreshGraph}>refresh</RefreshButton>
      <ForceGraph2D
        ref={fgRef}
        graphData={lastNetworkData}
        backgroundColor="#000003"
        linkColor={()=>'rgba(255,255,255,0.1)'}
        onNodeClick={handleNodeClick}
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
    </Container>
  )
}

export default App
