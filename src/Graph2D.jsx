import React from 'react';
import {ForceGraph2D} from 'react-force-graph';
import {useWindowSize} from '@react-hook/window-size';
import {isLinkBiDirectional} from './js/graphHandlers';
import {getPersonImage} from './js/serverApi.js';
import noImage from './assets/images/noImage.webp';
import styled from 'styled-components';

const ImageContainer = styled.div`
  position: absolute;
  display: none;
  /* width: 100px; */
  z-index: 10;
  top: 0; 
  border-radius: 10px;
  flex-direction: column;
  color: white;
`
const CustomImg = styled.img`
  /* width: 100%; */
  width: 100px;
  object-fit: cover;
  object-position: top;
  border-radius: 10px;
  aspect-ratio: 4/5;
`
const Contents = styled.div`
  width: fit-content;
  /* background-color: maroon; */
  backdrop-filter: blur(8px);  
`
const Text = styled.div`
  text-align: left;
  font-weight: 100;
  font-size: 12px;
`

const openChildWindow = (wikiUrl, windowFeatures) => {
  console.log('open', wikiUrl)
  window.open(`https://namu.wiki${wikiUrl}`, "aa", `width=800,height=1200,${windowFeatures}`);
}

let imageHash = {};
let noImageObj;
fetch(noImage)
.then(res => res.blob())
.then(blob => {
  noImageObj = URL.createObjectURL(blob)
})

const createImage = (src) => {
  const image = new Image();
  image.src = src;
  return image
}

const getImage = async (contentId) => {
  const cached = imageHash[contentId]
  console.log(cached)
  if(cached){
    return cached;
  }
  const imgBlob = await getPersonImage(contentId)
  if(imgBlob !== null){
    const objURL = URL.createObjectURL(imgBlob)
    imageHash[contentId] = objURL;
    return objURL
  }
  imageHash[contentId] = noImageObj;
  return null;
}

function positionElement(graph, x, y, element) {
  const {x:left, y:top} = graph.graph2ScreenCoords(x, y);
  element.style.top = `${top}px`;
  element.style.left = `${left + 10}px`;
  element.style.display = 'flex';
}

// 사용 예제

const showBox = (ctx, element, x, y) => {
  positionElement(ctx, x, y, element);
}


function Graph2D(props, graphRef) {
  const [width, height] = useWindowSize()
  const {graphData, expandNode} = props;
  const [nodeHovered, setNodeHovered] = React.useState(null);
  const [highlightNodes, setHighligntNodes] = React.useState(new Set());
  const [highlightLinks, setHighligntLinks] = React.useState(new Set());
  const [imgSrc, setImgSrc] = React.useState(noImageObj);
  // const fgRef = React.useRef(null);
  const imgRef = React.useRef(null);

  React.useEffect(() => {
    if(nodeHovered !== null){
      getImage(nodeHovered.id)
      .then(imgObjURL => {
        if(imgObjURL){
          // const image = createImage(imgObjURL)
          // const imageRatio = image.height / image.width;
          // const width = 100/globalScale;
          // const height = (100*imageRatio)/globalScale;
          setImgSrc(imgObjURL);
        }
      })

      showBox(graphRef.current, imgRef.current, nodeHovered.x, nodeHovered.y)
    } else {
      imgRef.current.style.display = 'none';
      setImgSrc(noImageObj)
    }
    
  }, [nodeHovered])
  const updateHighlight = React.useCallback(() => {
    setHighligntNodes(highlightNodes)
  }, [])

  const handleLeftClick = React.useCallback((node) => {
    const isForwarding = true;
    expandNode(node, isForwarding)
  }, [])
  const handleRightClick = React.useCallback((node) => {
    const isForwarding = false;
    expandNode(node, isForwarding)
  }, [])
  const handleLinkClick = React.useCallback((link, event) => {
    console.log('source:target-', link.source.id, link.target.id)
    const hasReverseLink = isLinkBiDirectional(link, graphData.links)
    const srcNode = link.source;
    const tgtNode = link.target;
    if(hasReverseLink){
      openChildWindow(tgtNode.wikiUrl, "right=0")
      setTimeout(() => {
        openChildWindow(srcNode.wikiUrl, "left=0")
      }, 5000)
    } else {
      openChildWindow(srcNode.wikiUrl, "right=0")
    }
  })

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
    setNodeHovered(node);
    // highlightNodes.clear();
    // if(node){
    //   highlightNodes.add(node);
    //   node.neighbors.forEach(neighbor => highlightNodes.add(neighbor));
    //   updateHighlight();
    // }
  }, [])

  return (
    <>
    <ForceGraph2D
      width={width}
      height={height}
      ref={graphRef}
      graphData={graphData}
      // backgroundColor="#000003"
      backgroundColor="transparent"
      linkColor={(link)=> highlightLinks.has(link)? 'rgba(234, 239, 44,0.5)': 'rgba(255,255,255,0.2)'}
      linkWidth={(link)=> highlightLinks.has(link) ? 5: 1}
      onNodeClick={handleLeftClick}
      onNodeRightClick={handleRightClick}
      onLinkClick={handleLinkClick}
      linkDirectionalArrowLength={0}
      linkDirectionalArrowRelPos={1}  
      linkDirectionalParticles={1}
      onEngineStop={() => console.log('engine stops')}
      // onEngineStop={() => graphRef.current.zoomToFit(100, 100)}
      onNodeDragEnd={node => {
        node.fx = node.x;
        node.fy = node.y;
        node.fz = node.z;
      }}
      onNodeHover={handleNodeHover}
      nodeCanvasObject={async (node, ctx, globalScale) => {
        const label = node.text;
        const fontSize = 14/globalScale;
        ctx.font = `300 ${fontSize}px SBAggroL`;
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
          ctx.lineWidth = 1 /globalScale;
          ctx.beginPath();
          ctx.strokeRect(node.x - bckgDimensions[0] / 2, node.y - bckgDimensions[1] / 2, ...bckgDimensions);
          ctx.strokeStyle = 'yellow';
        }
        // const isHoveredNode = nodeHovered === node
        // if(isHoveredNode){
        //   const imgObjURL = await getImage(node.id);
        //   if(imgObjURL){
        //     const image = createImage(imgObjURL)
        //     const imageRatio = image.height / image.width;
        //     const width = 100/globalScale;
        //     const height = (100*imageRatio)/globalScale;
        //     ctx.drawImage(image, node.x, node.y, width, height)
        //   }
        // }

        node.__bckgDimensions = bckgDimensions; // to re-use in nodePointerAreaPaint
      }}
      nodePointerAreaPaint={(node, color, ctx) => {
        ctx.fillStyle = color;
        const bckgDimensions = node.__bckgDimensions;
        bckgDimensions && ctx.fillRect(node.x - bckgDimensions[0] / 2, node.y - bckgDimensions[1] / 2, ...bckgDimensions);
      }}
    >
    </ForceGraph2D>
    <ImageContainer ref={imgRef}>
      <CustomImg src={imgSrc}></CustomImg>
      <Contents>
        {nodeHovered?.additionalInfo?.split('\n')
        .filter((info, index) => index < 10)
        .map((info) => {
          return <Text>{info}</Text>
        })}
      </Contents>
    </ImageContainer>
    </>
  )
}

export default React.memo(React.forwardRef(Graph2D));